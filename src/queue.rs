/// Async frame queue — mirrors the background streaming thread in libzedmd.
use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex};

pub(crate) struct Frame {
    pub id: u64,
    pub pixels: Vec<u16>,
}

struct State {
    queue: VecDeque<Frame>,
    next_id: u64, // ID to assign to the next pushed frame
    sent_id: u64, // ID of the last frame successfully sent to the device
    stopped: bool,
}

#[derive(Clone)]
pub(crate) struct FrameQueue {
    inner: Arc<(Mutex<State>, Condvar)>,
}

impl FrameQueue {
    pub fn new() -> Self {
        Self {
            inner: Arc::new((
                Mutex::new(State {
                    queue: VecDeque::with_capacity(1),
                    next_id: 1,
                    sent_id: 0,
                    stopped: false,
                }),
                Condvar::new(),
            )),
        }
    }

    /// Push a frame (latest-frame-wins). Returns the frame ID that can be
    /// passed to [`wait_for_frame`] to block until it has been sent.
    pub fn push(&self, pixels: Vec<u16>) -> u64 {
        let (lock, cvar) = &*self.inner;
        let mut state = lock.lock().unwrap();
        // If a frame is waiting unsent, it will be dropped — advance sent_id
        // to its ID so any wait_for_frame caller on that ID is unblocked.
        if let Some(dropped) = state.queue.pop_front() {
            if dropped.id > state.sent_id {
                state.sent_id = dropped.id;
            }
        }
        let id = state.next_id;
        state.next_id += 1;
        state.queue.push_back(Frame { id, pixels });
        cvar.notify_all();
        id
    }

    /// Block until a frame is available, then return it.
    /// Returns `None` when stopped and queue is empty.
    pub fn pop(&self) -> Option<Frame> {
        let (lock, cvar) = &*self.inner;
        let mut state = cvar
            .wait_while(lock.lock().unwrap(), |s| s.queue.is_empty() && !s.stopped)
            .unwrap();
        if state.stopped && state.queue.is_empty() {
            return None;
        }
        state.queue.pop_front()
    }

    /// Called by the background thread after a frame has been successfully sent.
    pub fn mark_sent(&self, id: u64) {
        let (lock, cvar) = &*self.inner;
        let mut state = lock.lock().unwrap();
        state.sent_id = id;
        cvar.notify_all();
    }

    /// Non-blocking check: returns `true` if the frame with the given ID
    /// (or a later one) has already been sent to the device.
    pub fn is_frame_sent(&self, id: u64) -> bool {
        let (lock, _) = &*self.inner;
        let state = lock.lock().unwrap();
        state.sent_id >= id || state.stopped
    }

    /// Block until the frame with the given ID (or a later one) has been sent.
    pub fn wait_for_frame(&self, id: u64) {
        let (lock, cvar) = &*self.inner;
        let _guard = cvar
            .wait_while(lock.lock().unwrap(), |s| s.sent_id < id && !s.stopped)
            .unwrap();
    }

    /// The number of frames sent so far — used for fps measurement.
    pub fn sent_count(&self) -> u64 {
        self.inner.0.lock().unwrap().sent_id
    }

    /// Signal the consumer thread to exit.
    pub fn stop(&self) {
        let (lock, cvar) = &*self.inner;
        let mut state = lock.lock().unwrap();
        state.stopped = true;
        cvar.notify_all();
    }
}

/// Async frame queue — mirrors the background streaming thread in libzedmd.
///
/// `render_rgb565_frame` on [`ZeDMDComm`] pushes frames here and returns
/// immediately. A dedicated thread pops frames and forwards them to the
/// device via [`SharedZeDMDComm::render_rgb565_frame`].
///
/// Like libzedmd the queue is bounded to 1 slot: if a new frame arrives
/// before the previous one has been sent, the old one is dropped. This
/// matches the "latest frame wins" semantics expected for real-time display.
use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};

/// A single pending frame.
pub(crate) struct Frame {
    pub pixels: Vec<u16>,
}

#[derive(Clone)]
pub(crate) struct FrameQueue {
    inner: Arc<(Mutex<State>, Condvar)>,
    /// Total frames actually sent to the device by the background thread.
    sent: Arc<AtomicU64>,
}

struct State {
    queue: VecDeque<Frame>,
    stopped: bool,
}

impl FrameQueue {
    pub fn new() -> Self {
        Self {
            inner: Arc::new((
                Mutex::new(State {
                    queue: VecDeque::with_capacity(1),
                    stopped: false,
                }),
                Condvar::new(),
            )),
            sent: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Push a frame. If the queue already holds a pending frame it is replaced
    /// (drop-oldest policy, matching libzedmd's `FillDelayed` / queue-full path).
    pub fn push(&self, pixels: Vec<u16>) {
        let (lock, cvar) = &*self.inner;
        let mut state = lock.lock().unwrap();
        if !state.queue.is_empty() {
            state.queue.clear(); // drop stale frame
        }
        state.queue.push_back(Frame { pixels });
        cvar.notify_one();
    }

    /// Block until a frame is available, then return it.
    /// Returns `None` if the queue has been stopped.
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

    /// Signal the consumer thread to exit.
    pub fn stop(&self) {
        let (lock, cvar) = &*self.inner;
        let mut state = lock.lock().unwrap();
        state.stopped = true;
        cvar.notify_all();
    }

    /// Called by the background thread after each successful device send.
    pub fn mark_sent(&self) {
        self.sent.fetch_add(1, Ordering::Relaxed);
    }

    /// Total frames sent to the device so far.
    pub fn sent_count(&self) -> u64 {
        self.sent.load(Ordering::Relaxed)
    }
}

//! Cooperative cancellation and progress reporting for long operations.

use std::sync::atomic::{AtomicBool, Ordering};

/// Returned when an operation stopped because cancellation was requested.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cancelled;

impl std::fmt::Display for Cancelled {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "operation cancelled")
    }
}

impl std::error::Error for Cancelled {}

type ProgressFn<'a> = &'a (dyn Fn(f32, &str) + Sync);

/// Handle passed to long-running operations. Operations poll [`Control::check`] between steps.
#[derive(Clone, Copy)]
pub struct Control<'a> {
    cancel: Option<&'a AtomicBool>,
    progress: Option<ProgressFn<'a>>,
}

impl<'a> Control<'a> {
    /// A control that never cancels and ignores progress.
    pub const fn none() -> Self {
        Self { cancel: None, progress: None }
    }

    pub fn new(cancel: &'a AtomicBool, progress: ProgressFn<'a>) -> Self {
        Self { cancel: Some(cancel), progress: Some(progress) }
    }

    pub fn with_cancel(cancel: &'a AtomicBool) -> Self {
        Self { cancel: Some(cancel), progress: None }
    }

    pub fn is_cancelled(&self) -> bool {
        self.cancel.is_some_and(|c| c.load(Ordering::Relaxed))
    }

    /// `Err(Cancelled)` once cancellation has been requested.
    pub fn check(&self) -> Result<(), Cancelled> {
        if self.is_cancelled() {
            Err(Cancelled)
        } else {
            Ok(())
        }
    }

    /// Reports progress in `0.0..=1.0` with a short description.
    pub fn report(&self, fraction: f32, what: &str) {
        if let Some(p) = self.progress {
            p(fraction.clamp(0.0, 1.0), what);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    #[test]
    fn cancellation_and_progress() {
        let flag = AtomicBool::new(false);
        let log = Mutex::new(Vec::new());
        let progress = |f: f32, s: &str| log.lock().unwrap().push((f, s.to_string()));
        let c = Control::new(&flag, &progress);
        assert!(c.check().is_ok());
        c.report(0.5, "half");
        flag.store(true, Ordering::Relaxed);
        assert_eq!(c.check(), Err(Cancelled));
        assert_eq!(log.lock().unwrap().as_slice(), &[(0.5, "half".to_string())]);
        assert!(Control::none().check().is_ok());
    }
}

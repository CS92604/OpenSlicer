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
    /// Progress reported by this handle maps to `offset + scale * fraction` of the parent's range.
    offset: f32,
    scale: f32,
}

impl<'a> Control<'a> {
    /// A control that never cancels and ignores progress.
    pub const fn none() -> Self {
        Self { cancel: None, progress: None, offset: 0.0, scale: 1.0 }
    }

    pub fn new(cancel: &'a AtomicBool, progress: ProgressFn<'a>) -> Self {
        Self { cancel: Some(cancel), progress: Some(progress), offset: 0.0, scale: 1.0 }
    }

    pub fn with_cancel(cancel: &'a AtomicBool) -> Self {
        Self { cancel: Some(cancel), progress: None, offset: 0.0, scale: 1.0 }
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
            p((self.offset + self.scale * fraction.clamp(0.0, 1.0)).clamp(0.0, 1.0), what);
        }
    }

    /// A handle for a stage that occupies `lo..hi` of this handle's progress range. Cancellation is shared.
    pub fn sub(&self, lo: f32, hi: f32) -> Control<'a> {
        Control { offset: self.offset + self.scale * lo, scale: self.scale * (hi - lo), ..*self }
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

    #[test]
    fn sub_ranges_scale_and_compose() {
        let flag = AtomicBool::new(false);
        let log = Mutex::new(Vec::new());
        let progress = |f: f32, _: &str| log.lock().unwrap().push(f);
        let c = Control::new(&flag, &progress);
        let stage = c.sub(0.2, 0.6);
        stage.report(0.5, "x");
        stage.sub(0.5, 1.0).report(1.0, "y");
        stage.report(5.0, "clamped");
        let got = log.lock().unwrap().clone();
        assert_eq!(got.len(), 3);
        for (g, want) in got.iter().zip([0.4, 0.6, 0.6]) {
            assert!((g - want).abs() < 1e-6, "{got:?}");
        }
        flag.store(true, Ordering::Relaxed);
        assert!(stage.sub(0.0, 0.5).check().is_err(), "cancellation is shared with sub-ranges");
    }
}

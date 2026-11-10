# SPECTER

Open specter_dashboard.html in desktop Chrome or Edge. Keep all three JavaScript files beside it.

- specter_dashboard.html: dashboard UI
- specter-kinematics.js: calibrated motion calculations
- specter-nexus.js: firmware integration and diagnostics
- specter-assistant.js: assistant behavior
- specter_arm_smooth.ino: firmware source; uploading is a separate manual action

USB uses 460800 baud. The firmware repair lock and switch safety logic remain authoritative. GPIO labels use configuration received from the connected firmware.

Cleanup archived old backups, screenshots, tests, reports, build outputs, and the unused credential text file in ../SPECTOR-cleanup-backup-2026-11-09. Test scripts must be restored beside the source files to run them again. The backup manifest records original paths.

Music dance: keep specter-dance-track.js and juliush-slow-piano-waltz-293174.mp3 beside the dashboard. Automation > Dance > Play + dance to beat uses the analysed 95.7 BPM timing estimate. Busy beats are skipped; base scale and existing motion protections still apply.

Stepper pin swap: base STEP=GPIO18 / DIR=GPIO19 (X); elbow STEP=GPIO21 / DIR=GPIO22 (Y). Firmware source updated only; not uploaded or hardware verified. Other pin assignments and safety logic unchanged by this swap.

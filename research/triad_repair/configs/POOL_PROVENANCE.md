# Diagnostic common pool

The 40 IDs are the CURRENT-POSE SDF shortlist logged in generation 1 of
`gate4_paired_longitudinal/run.log`, before future-event selection. No
robot-feasible, authority, terminal, or completion labels are used to select IDs.
They are frozen for the three longitudinal implementation diagnostics only.
This is not a preregistered population H1 pool. For H1, derive/freeze pools on
shared initial observations using this same outcome-blind current-state rule;
verify identical pool hashes before running paired treatments. Do not use
the future-event shortlist or selected predictive action as the reactive pool.
The 0.20 s lookahead is a diagnostic setting, not a tuned/validated optimum.

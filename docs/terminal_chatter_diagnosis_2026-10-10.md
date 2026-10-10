# Follow-up: current terminal design is not an accepted performance improvement

The user's rerun exactly reproduces `terminal_20261010_085317` tracking metrics.
Against their ZENG run, PMPC is worse in both DLC and J-turn tracking. A reduction
in late steering roughness relative to the previous failing-QP version is NOT
evidence that steering chatter has been solved. No further controller changes
or tuning changes were made during this diagnostic pass.

## Read-only saved-log audit

Script: `scratchpad/terminal_improve_20261009/inspect_terminal_chatter.m`.
Source: the saved `terminal_20261010_085317` logs and parameter snapshot.
Brake moment is reconstructed using the exact lower-level allocator matrix
and logged four-wheel commands, NOT measured tire forces or actual CarSim yaw
moment. Positive denotes left-yaw under the controller convention.

| Time (s) | Requested DB mean (Nm) | Allocated command mean (Nm) | Allocated positive >50 Nm | Allocated negative <-50 Nm |
|---|---:|---:|---:|---:|
| 11.5–13 | -379.0 | -375.1 | 11.3% | 58.0% |
| 13–14 | -246.9 | -248.7 | 15.0% | 51.0% |
| 14–17 | -444.5 | -445.9 | 14.0% | 84.0% |

The 13–14 s request range is [-1531.3,894.4] Nm and allocated-command range
[-1530.8,868.2] Nm. Thus positive requests are present but brief; the dominant
negative request originates in the upper-level optimization, rather than a
systematic left/right sign inversion in the allocator. Actual delivered tire
moment is not inferred from this command reconstruction alone.

There are 33 terminal validity transitions in the 20 s run. Among 138 steering
command steps above 3 deg per 10 ms, 56 occur within +/-30 ms of a validity
transition. These neighborhoods occupy only 6.2% of noninitial samples.
This is strong temporal association, not proof that transitions explain every
spike: 82 large steps occur outside these neighborhoods.

## Structural concern requiring redesign, not larger weights

The current terminal core is constructed around a zero-steady-DB equilibrium,
zero ey, and the midpoint of instantaneous damper moment bounds. It is a
restricted sufficient recovery set, not the full safe recoverable set.
Backup input, input-rate and core-distance inequalities share the same safety
slacks as physical vehicle constraints. Therefore reducing those slacks can
favor proximity to this particular moving core over otherwise safe tracking.
Zero DB is only the center, not a hard prohibition on nonzero DB.

Invalid cores remove an entire set of terminal rows. The changing core,
its validity transitions and changing priority weights are plausible sources
of discontinuous optimal commands. The seven-state frozen-Vx predictor also
does not credit DB-induced deceleration. Neither increasing R1 alone nor
forcing a left-yaw sign is a principled solution to these issues.

The nonterminal repaired baseline remains available (DLC RMS 0.297228 m,
J-turn RMS 0.393520 m); disabling terminals recovers DLC performance but does
not solve J-turn. The bound-order repair should be retained independently of
whether this terminal construction is withdrawn or redesigned.

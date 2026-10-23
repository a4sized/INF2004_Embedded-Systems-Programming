# System fault integration checks

Status: **HARDWARE / TEAM INTEGRATION NOT RUN** (23 October 2026). This is a
test plan for Buddy 1 and the controller/motion owners.

The IR branch now contains Buddy 1's Gatekeeper API and `FAULT_IR_SENS`.
The isolated sensing image links the actual Subsystem 1 source. On GPIO
initialization failure, Buddy 3 calls `s1_report_fault(FAULT_IR_SENS)` and
`s1_csv_log_event`, then reasserts the fault every 250 ms while the sensing
task remains halted. Buddy 3 does not write `fault_flag`, clear the fault,
or command motors. Sample gaps above 10 ms remain diagnostic counters;
barcode overflow invalidates a capture rather than raising a system fault.

Buddy 2's `798da4b` update now reads Buddy 1's persistent telemetry
`fault_mask` every 20 ms, so a Buddy 3 task-context report remains visible
after the LED task drains `flg_s1_isr_fault`. This resolves the earlier
source-level report/poll race for `FAULT_IR_SENS`; a motor halt still needs
hardware fault injection before FLT-01 can pass. `s1_clear_fault` clears the
telemetry mask, while Buddy 3 reasserts an unresolved GPIO-init failure.
The reset/clear path remains untested.

The same Buddy 2 change stops checking the separate global `fault_flag`.
`motion_trigger_safety_halt` still writes that flag. Its immediate stop is
implemented, but persistence and release behavior after a direct motor
safety halt need Buddy 1/2 review and testing.

| ID | Case | Expected result | Status |
|---|---|---|---|
| FLT-01 | Inject an IR initialization/confirmed sensor failure | `FAULT_IR_SENS` reaches Buddy 1, remains active while failed, and the agreed controller response occurs | SOURCE WIRED; HARDWARE NOT RUN |
| FLT-02 | Raise IR and motor faults close together | Both remain visible; neither producer erases the other | NOT RUN |
| FLT-03 | Issue STOP/ESTOP during sensing or barcode capture | Movement stops and pending capture/navigation work cannot restart it | NOT RUN |
| FLT-04 | Attempt CLEAR while the source failure remains | Fault stays active; movement cannot resume | NOT RUN |
| FLT-05 | Perform the approved reset after source recovery | Only the authorized owner clears the latch; IR readiness is re-established before following | NOT RUN |
| FLT-06 | Interrupt-context fault notification | ISR performs no blocking or mutex operation; task-side path observes the event | SOURCE FIX MERGED; HARDWARE NOT RUN |
| FLT-07 | Wi-Fi disconnect during autonomous navigation | Network recovers independently; connectivity loss alone does not stop onboard navigation | NOT RUN |
| FLT-08 | Trigger Buddy 2 direct safety halt, then set speed intent to zero | Global motor fault remains latched until approved clear/reset | SOURCE REVIEW NEEDED; NOT RUN |

Before marking these complete, agree the IR clear/reset policy, fault-mask
to motor-halt path, stop acknowledgement, and Vehicle Controller owner.

The local status host test, isolated ARM build, and standard team ARM build
passed after merging `798da4b`. They cannot prove physical failure
detection, cloud visibility, motor response or reset.

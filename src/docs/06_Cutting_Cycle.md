# Cutting Cycle Sequence

The cutting cycle follows these specific steps:

## Main Cutting Sequence

1. **Cycle Start**: Initialize cycle variables and turn on yellow LED
2. **Clamp Verification**: Confirm both position and secure wood clamps are extended
3. **Cut Operation**: Move cut motor to 8.5-inch position
4. **Wood Suction Check**: Check WAS_WOOD_SUCTIONED_SENSOR when the cut motor reaches exactly 0.3 inches into the cut. If sensor indicates a problem (is LOW), immediately trigger an error condition and halt the cutting operation.
5. **Signal Transfer Arm**: Send signal to the Transfer Arm (TA) when the cut motor reaches 7.2 inches (0.5 inches before end of travel)
6. **YESorNO_WOOD_SENSOR Detection**: Check if wood is present using the YESorNO_WOOD_SENSOR for subsequent operations

## Wood Present Sequence (YESWOOD_STATE)

7. **Initial Return**: Start both motors moving to their home positions
8. **Initial Clamp Management**: Retract the position clamp prior to returning home, while keeping wood secure clamp extended
9. **Complete Return**: Continue movement until both motors reach their home position
10. **Home Position Check**: Wait 50ms after the cut motor reaches position zero, then verify the cut motor position switch reads HIGH. If not, trigger the CUT_MOTOR_HOME_ERROR_STATE
11. **Position Clamp Engagement and Secure Clamp Release**: Extend position clamp and simultaneously retract secure wood clamp once position zero is reached, allowing the position clamp to drag the wood forward without restriction
12. **Positioning Operation**: Move position motor to 3.45 inches for next cut, ensuring it first returns to zero
13. **Cycle Complete**: Return to READY_STATE

## No Wood Sequence (NOWOOD_STATE)

7. **Clamp Management**: Retract wood secure clamp
8. **Home Return**: With the position clamps still extended, Both motors return to home position
9. **Position Clamp Release**: Retract position clamp
10. **Cycle Complete**: Set no-wood flag and return to READY_STATE. The system still allows one final cut to be performed before NOWOOD_STATE movements are executed

## Key Position Constants

```cpp
// Specific positions for the cutting cycle
const float WOOD_SUCTION_CHECK_POSITION = 0.3;  // inches - Position to check suction
const float TRANSFER_ARM_SIGNAL_POSITION = 7.2; // inches - Position to signal transfer arm
const float CUT_COMPLETE_POSITION = 8.5;        // inches - Full travel of cut motor
const float POSITION_MOTOR_RETRACT_DISTANCE = 0.1; // inches - Distance at which to retract position clamp
```

## Implementation Considerations

- The cutting cycle is designed to be continuous if the Cycle Switch remains activated
- Critical safety checks occur at specific positions during the cutting operation
- Proper clamp sequencing is essential for safe operation
- The cycle automatically determines if wood is present for the next cycle
- A complete cutting cycle includes the return to home position and preparation for the next cut 
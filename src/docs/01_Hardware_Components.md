# Hardware Components

## Motors
1. **Cut Motor** - Controls the cutting blade movement across the 8.5-inch travel path
2. **Position Motor** - Controls the wood positioning for precise movement across a 3.45-inch travel path

## Clamps
1. **Position Clamp** - Secures the wood during positioning operations and provides additional support during cutting
2. **Secure Wood Clamp** - Firmly holds the wood in position during cutting operations

## Switches and Sensors
1. **Cut Motor Position Switch** - Limit switch that detects when the cut motor is at home position (HIGH when triggered)
2. **Position Motor Position Switch** - Limit switch that detects when the position motor is at home position (HIGH when triggered)
3. **Reload Switch** - Activates reload mode for loading new wood (HIGH when triggered)
4. **Cycle Switch** - Initiates the cutting cycle (HIGH when triggered)
5. **YESorNO_WOOD_SENSOR** - Detects the presence of wood (LOW when wood is present)
6. **WAS_WOOD_SUCTIONED_SENSOR** - Verifies proper wood suction during cutting operations (HIGH when suction is correct, LOW indicates failure/error)

## LED Indicators
1. **Red LED** - Error indicator - blinks with pattern indicating specific error type
2. **Yellow LED** - Operation in progress indicator
3. **Green LED** - Ready indicator - illuminates when system is ready for cutting cycle
4. **Blue LED** - Setup/special mode indicator - illuminates during startup, when no wood is present, and during reload mode 
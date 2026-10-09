# App Preferences

- This repository is an FTC Robot Controller project written in Java.
- Preserve existing OpMode controls and Robot Configuration device names unless a change is explicitly requested.
- For servo test OpModes, report the configured servo type and relevant live gamepad state in telemetry.
- Support both standard positional servos and continuous-rotation servos when the hardware type is uncertain.
- Keep robot control loops responsive by yielding with `idle()`.
- Validate Java changes with `.\gradlew.bat :TeamCode:compileDebugJavaWithJavac`.
- Do not commit or push changes without explicit confirmation.

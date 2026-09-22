# Integration status and verification

Reviewed September 21, 2026 against [the implementation plan](../PedroSwerving/PEDRO_INTEGRATION_PLAN.md).

The software integration is implemented, desktop-tested, and builds with both local Pedro artifacts and published 3.0.1. Pinpoint direction and heading commissioning has been accepted for the current robot configuration. Foresight model characterization remains a separate gate before path autonomy.

## Verification evidence

| Check | Result |
| --- | --- |
| Pedro `:core:test` | 7 compatibility-contract tests passed |
| Pedro `:core:spotlessCheck` | Passed |
| Pedro `:revhub:assembleRelease` | Release AAR built |
| Pedro `:revhub:lintRelease` | Passed, no lint issues |
| Robot `:TeamCode:lintDebug` | Passed; dependency native-library alignment and unused SDK resource warnings remain |
| Robot against local Pedro JAR/AAR | 103 unit tests passed; debug APK assembled, including D8 and duplicate-class checks |
| Robot against published Pedro 3.0.1 | 103 unit tests passed; debug APK assembled, including D8 and duplicate-class checks |
| Runtime dependency graph | `core:3.0.1`, `revhub:3.0.1`, Kotlin stdlib 2.3.21; FTC artifacts remain 12.0.0 |

Tests include the existing control/encoder/snapshot suite plus both-pod motor mapping, live autonomous caps, zero/BRAKE behavior, initialization cancellation, fresh Start drift, partial initialization/output failure, fault latching, loop overruns, hub recovery, guarded Pinpoint status/timing/finite values, explicit/default follower timing, Foresight frame conversion, and terminal accuracy classification. The runtime tests inject a host and motor boundaries into the actual state machine; they do not need Android stub methods to imitate hardware.

Warnings remain for Java 8 source/target on JDK 21 and legacy Gradle APIs used by the existing build plugins/FTC build. Robot lint also identifies 16 KB alignment in bundled FTC/OpenFTC native libraries and unused SDK resources. These dependency warnings were retained without changing the pinned FTC stack; compatibility with an Android device requiring 16 KB native-library pages is not established. No wrapper, FTC SDK, or compile SDK upgrade was needed. Same-thread software cannot interrupt a blocked SDK call or guarantee delivery of a failed motor stop; physical fault injection is still required.

## Reproduce

Configure each repository's `local.properties` or `ANDROID_HOME` for the Android SDK. Pedro builds with platform 35; the robot retains compile SDK 30. Use JDK 21 locally (the robot already has daemon criteria for JDK 21); Java source/target remain 8.

From the Pedro repository on Windows:

```powershell
./scripts/verify-projects.ps1 -BuildJdk 'YOUR_JDK_21_DIRECTORY'
```

The script verifies Pedro, runs robot checks against local artifacts, then restores the default published dependency for the final APK/reports. Its `-RobotRepo` parameter supports arbitrary checkout locations. It does not publish or install anything on the robot.

Equivalent individual commands:

```powershell
# Pedro checkout
./gradlew.bat :core:test :core:jar :core:spotlessCheck :revhub:assembleRelease :revhub:lintRelease
# Robot checkout: the property points to the Pedro checkout, after the preceding build.
./gradlew.bat '-PpedroLocalArtifacts=../PedroSwerving' :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
# Normal deployment build: omit the property to use published 3.0.1.
./gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug :TeamCode:lintDebug
```

On Unix replace `./gradlew.bat` with `./gradlew`. `pedroLocalArtifacts` is optional and replaces both Pedro modules with local files for that invocation; it does not append duplicate classes or import Pedro build plugins. Rebuild local artifacts before using this option. Normal published builds need only the robot checkout.

Outputs:

- Robot APK: `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk` (the exact output name is listed by `output-metadata.json`).
- Robot tests: `TeamCode/build/reports/tests/testDebugUnitTest/index.html`.
- Pedro tests: `core/build/reports/tests/test/index.html`.
- Pedro library: `revhub/build/outputs/aar/revhub-release.aar` and `core/build/libs/core-3.0.1.jar`.
- Dependency inspection: `./gradlew.bat :TeamCode:dependencies --configuration debugRuntimeClasspath`.

## Ownership and behavior

| Source | Responsibility |
| --- | --- |
| `DifferentialSwerveRuntime` | INIT/READY/ARMED/fault/Stop state, sample timing, output authorization, autonomous cancellation and TeleOp recovery |
| `DifferentialSwerveHardware`, `SwerveHubSession`, `Cleanup` | SDK acquisition, changed-value PIDF, validated bulk snapshots, independent cleanup attempts/cache restoration |
| `DifferentialPod` | Existing encoder/alignment/controller composition, one right-pod negate-and-swap, per-write authorization, finite output cap, immediate zero |
| `DifferentialSwerveDrivetrain`, `PedroSwerveMath` | Forward/left/CCW contract, existing kinematics, linear wheel-envelope allocator, zero/BRAKE policy |
| `GuardedPinpointLocalizer`, `PinpointSettings` | Shared sensor configuration, stationary calibration, READY/timing/finite-state checks, one update per follower cycle |
| `SafePedroFollower`, `PedroAutoDrive` | Normal Pedro API under a guarded cycle, immediate stop, fault cancellation, cleanup-protected lifetime, disarmed model validation |
| `PedroFollowerConfig`, `ManualOnlyAlgorithm` | Complete model/configuration validation; manual commissioning before path-model approval |
| `PedroPathResult`, `PedroPathTestBase` | Independent terminal acceptance, 10-second deadline and 2-second no-progress detection after steering allowance |
| `DifferentialSwerveTeleOp` | Existing radial/cubic input shaping, full-scale TeleOp profile, neutral recovery, numeric Dashboard diagnostics |
| Pedro commissioning OpModes | Selected-pod static angles; manual drive; signed characterization logs; disabled line/curve/heading/template paths |

All primary sources are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`; unit tests are under the matching `src/test/java` package.

The robot's coordinate contract is X forward, Y left, yaw CCW, inches/radians. The calibrated pod tracker stays CW-positive internally. Conversion occurs exactly once at the drivetrain boundary. The published generic `Swerve` is intentionally unused: its FLOAT and cosine-scaling behavior conflicts with this robot's lifecycle and controller.

Nonzero outputs require the correct phase and a fresh sample, checked again before each motor write. Direct pod failures also stop and latch the owning runtime. Completely zero chassis demand clears steering output and commands zero velocity; an individual zero-speed pod in a moving chassis retains its target. TeleOp uses its established 1.0 drive profile; autonomous uses 0.15 drive, 0.20 turn envelope, and 0.35 combined motor cap. Live tuning cannot bypass that cap. Startup alignment has a 2-degree tolerance, 100 ms settle, and 5-second deadline.

Runtime reads retry at most three times within 150 ms, stopping before retries. Loops/sample ages above 250 ms fail closed. TeleOp waits for neutral controls after recovery; autonomous and the armed static-angle diagnostic cancel. Analog feedback seeds the tracker and checks recovery; runtime azimuth remains quadrature-only.

## Published follower behavior

`Follower.stop()` in Pedro 3.0.1 only changes state; the robot's `SafePedroFollower.stop()` sends immediate motor zero. Both `update()` overloads acquire fresh hardware feedback. An explicit delta affects algorithm timing only, never the hardware clock or output-age checks. A skipped-output cycle, including FOLLOW-to-HOLD transition, cannot leave a previous command active.

Foresight advances at the parametric endpoint even if terminal heading or velocity is outside configured tolerances. `isBusy()` is not a complete lifecycle signal, and `distanceToEndpoint()` becomes zero after the tracker is cleared. The path tests retain the requested endpoint and evaluate actual pose/velocity independently. Acceptance requires <=0.5 in translation, <=3 degrees heading, <=1 in/s translation speed, and <=5 degrees/s angular speed. Outside-tolerance/invalid completion latches a fault. The overall deadline is independent of `timeoutConstraint` (milliseconds). These diagnostics report accuracy; they do not automatically add an uncommissioned final pose-hold maneuver.

## Physical work still required

Follow [HUMAN_TASKS.md](HUMAN_TASKS.md), preserving its actual measurement records:

1. Repeat INIT/Start/Stop and low-speed TeleOp regressions after the shared-runtime refactor; verify Dashboard signals and hardware motor signs.
2. Pinpoint X/Y directions, CCW heading/rate, field frame, distance scale, and rotation behavior are accepted for the current robot configuration; retain monitoring during Pedro manual commissioning.
3. Run selected-pod static angles, then bounded Pedro manual drive, including delayed/failed feedback and release-to-zero behavior.
4. Capture characterization CSV, fit loaded speed/deceleration/braking models in the documented units, and enter the independent recorded drive scale. Set `MODEL_VERIFIED` only after those records exist.
5. Enable one disabled path diagnostic at a time: lines, curves, heading changes and wrap cases. Record completion reasons, terminal error/velocity, saturation, battery condition, and repeatability before enabling the template for competition.

`DIRECTIONS_VERIFIED=true` and the heading convention is `COUNTERCLOCKWISE_POSITIVE` for the commissioned robot. `MODEL_VERIFIED=false` and the `@Disabled` path annotations remain intentional until measured Foresight characterization is entered.

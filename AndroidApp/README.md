# AutoNav — Android operator app

Vite + React 18 + TypeScript app, wrapped with [Capacitor](https://capacitorjs.com/) into an
installable Android APK. Operates a robot assigned to the signed-in user via PlatformServer's
`/ws/control` teleop relay. Mirrors `PlatformApp`'s RTK Query / React Router / react-hook-form
conventions (see `src/` for the feature-sliced layout).

## Develop

```bash
npm install
cp .env.example .env   # fill in VITE_API_BASE_URL etc. for your environment
npm run dev
```

## Build + typecheck

```bash
npm run build       # tsc -b && vite build
npm run typecheck   # tsc -b --noEmit
npm run lint
```

## Android (Capacitor)

Rebuild the web bundle, sync it into the native project, then build the APK:

```bash
npm run build
npx cap sync android
cd android && ./gradlew assembleDebug
```

This produces `android/app/build/outputs/apk/debug/app-debug.apk`. `npx cap run android` does
the same build and installs it onto a connected device/emulator in one step — equivalent to:

```bash
adb install -r android/app/build/outputs/apk/debug/app-debug.apk
```

```bash
adb logcat --pid=$(adb shell pidof -s in.autonav.app) -v time
```

(`-r` reinstalls over an existing install, keeping it from failing if the app's already there.)

`adb devices -l` lists connected devices; if none shows up, reconnect the USB cable or
re-approve the "Allow USB debugging" prompt on the phone.

Gradle itself must run under **JDK 21** — this is pinned in `android/gradle.properties`
(`org.gradle.java.home`) so it's independent of whatever Java your shell's `PATH`/`JAVA_HOME`
default to (AGP 8.x's Gradle plugin fails under JDK 25 with
`Unsupported class file major version 69`). No manual JDK switching needed as long as that
property stays pointed at a real JDK 21 install.

See `capacitor.config.ts` for `appId`/`webDir`. The generated `android/` native project is
committed (it carries hand-made native config — network security, manifest permissions) and is
kept in sync with `npx cap sync android` after any web build.

### Google Sign-In setup (one-time, manual)

Native Google Sign-In needs an **Android** OAuth client registered in Google Cloud Console
(same project as the existing web client used for `VITE_GOOGLE_WEB_CLIENT_ID`), bound to this
app's package (`in.autonav.app`) and signing-key SHA-1. Get the debug keystore's SHA-1 with:

```bash
keytool -list -v -keystore ~/.android/debug.keystore -alias androiddebugkey -storepass android -keypass android
```

Register a separate Android OAuth client for a release-signed build's SHA-1 once that keystore
exists. The `clientId` passed to `GoogleSignIn.initialize()` in code stays the **web** client ID
either way — the Android client above only needs to exist in Google Cloud Console, it's never
referenced directly in app code.

## Environment variables

See `.env.example`. `VITE_API_BASE_URL` / `VITE_WS_BASE_URL` point at PlatformServer; for a
physical device in local dev, use your dev machine's LAN IP, not `localhost`.

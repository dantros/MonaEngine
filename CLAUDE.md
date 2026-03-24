# MonaEngine — CLAUDE.md

## Project Overview

MonaEngine is a modern C++20 3D game engine built for educational and research purposes. It features rendering (OpenGL 4.5), physics (Bullet), audio (OpenAL-Soft), skeletal animation, inverse kinematics, and an optional Entity-Component-System via EnTT. The primary development platform is Windows with MSVC 2022; Linux is also supported.

---

## Build System

**Requirements:**
- CMake 3.15+ (3.20+ recommended)
- C++20 capable compiler (MSVC 2022 on Windows, GCC/Clang on Linux)
- All third-party dependencies are Git submodules — always run `git submodule update --init --recursive` after cloning

**Predefined CMake presets (from `CMakePresets.json`):**

| Preset | Generator | Config |
|---|---|---|
| `vs-debug` | Visual Studio 17 2022 | Debug |
| `ninja-debug` | Ninja | Debug |
| `ninja-release` | Ninja | Release |
| `linux-debug` | Unix Makefiles | Debug |
| `linux-release` | Unix Makefiles | Release |

**Building with a preset:**
```bash
cmake --preset vs-debug
cmake --build build/vs-debug
```

**Manual CMake:**
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
cmake --build .
```

**Output:** Static library `MonaEngine.lib` (Windows) or `MonaEngine.a` (Linux), installed alongside public headers and `EngineAssets/`.

**Key CMake options:**
- `TRACY_ENABLE` — Enable Tracy profiler instrumentation (OFF by default)
- `CMAKE_INSTALL_PREFIX` — Installation directory

**Compiler flags set by CMake:**
- MSVC: `/wd5033 /utf-8`
- Linux/GCC: `-Wno-narrowing`
- All platforms: position-independent code enabled

---

## Architecture

### Entry Point Pattern

Users subclass `Mona::Application` and implement three virtual methods:

```cpp
class MyApp : public Mona::Application {
public:
    void UserStartUp(Mona::World& world) noexcept override { /* init */ }
    void UserUpdate(Mona::World& world, float timestep) noexcept override { /* per-frame */ }
    void UserShutDown(Mona::World& world) noexcept override { /* cleanup */ }
};

int main() {
    MyApp app;
    Mona::Engine engine(app);
    engine.StartMainLoop();
}
```

### Core Hierarchy

```
Engine
└── World (singleton per application)
    ├── GameObjectManager  (generational handles)
    ├── ComponentManagers  (10 types, contiguous array storage)
    ├── Renderer           (OpenGL 4.5)
    ├── PhysicsCollisionSystem (Bullet Physics 3)
    ├── AudioSystem        (OpenAL-Soft)
    ├── AnimationSystem
    ├── IKNavigationSystem (inverse kinematics)
    ├── EventManager       (pub/sub, generational subscription handles)
    ├── Input / Window     (GLFW, pimpl abstraction)
    └── ECSHandler         (EnTT wrapper, opt-in via World::EnableECS())
```

### Subsystems (`source/`)

| Directory | Purpose |
|---|---|
| `Animation/` | Skeletal animation, clips, state machine, cross-fading |
| `Audio/` | 3D audio, priority-based source management |
| `CharacterNavigation/` | IK system, trajectory generation, height maps |
| `Core/` | Config singleton, logging (spdlog), assertions |
| `DebugDrawing/` | Physics and IK shape visualization |
| `ECS/` | Optional EnTT wrapper (`MonaECS` namespace) |
| `Event/` | Typed pub/sub event system |
| `PhysicsCollision/` | Bullet3 rigid bodies, raycasting, collision callbacks |
| `Platform/` | GLFW window/input (pimpl pattern) |
| `Rendering/` | Materials, shaders, textures, mesh loading (Assimp) |
| `Utilities/` | Camera controllers, engine helper objects |
| `World/` | GameObject system, component managers, engine lifecycle |
| `Application/` | `Application` base class |

---

## Component System

Components are added/removed through the `World`:

```cpp
auto go = world.CreateGameObject<MyGameObject>();
auto handle = world.AddComponent<Mona::TransformComponent>(go, ...);
```

**Available component types:** `TransformComponent`, `CameraComponent`, `StaticMeshComponent`, `SkeletalMeshComponent`, `RigidBodyComponent`, `AudioSourceComponent`, `DirectionalLightComponent`, `PointLightComponent`, `SpotLightComponent`, `IKNavigationComponent`.

- Components declare required siblings via a `dependencies` template parameter — validated at add-time.
- Component storage is contiguous (cache-friendly). Handles are generational for safety.
- Lifetime policies control custom cleanup on removal (used by RigidBody, AudioSource, IKNavigation).

---

## Rendering

- **API:** OpenGL 4.5 (required — no fallback)
- **Material types:** `UnlitFlat`, `UnlitTextured`, `DiffuseFlat`, `DiffuseTextured`, `PBRFlat`, `PBRTextured`
- **Scene limits:** 1 directional light, 3 point lights, 3 spot lights
- **Skinning:** Bone matrix palette, max 70 bones
- **Mesh loading:** Assimp (FBX, GLTF, OBJ); built-in primitives: Cube, Sphere, Plane, Axis
- **Texture loading:** STB Image (PNG, JPG, TGA)
- **Singletons:** `MeshManager`, `TextureManager`

---

## Physics

- **Engine:** Bullet Physics 3
- **RigidBody types:** Static, Dynamic, Kinematic
- **Collision shapes:** Box, Sphere, Capsule, Cone, Cylinder
- **Default gravity:** `(0, 0, 0)` — must be set explicitly
- **Raycast:** single closest-hit and all-hits variants
- **Callbacks:** `StartCollision` / `EndCollision` events via `EventManager`
- **Debug draw:** Toggle via `DebugDrawingSystem_physics`

---

## Audio

- **Backend:** OpenAL-Soft (dynamic library, LGPL)
- **Channels:** 32 OpenAL sources by default (configurable in `config.json`)
- **Priority levels:** Low, Medium, High, Critical — used when sources are exhausted
- **File format:** WAV only (loaded via dr_wav)
- **Singleton:** `AudioClipManager`

---

## Animation

- **Loading:** Assimp skeletons and animation clips
- **Singletons:** `SkeletonManager`, `AnimationClipManager`
- **Controller:** State-machine with cross-fade transitions
- **Runtime query:** World-space joint poses accessible at runtime

---

## Configuration & Assets

Assets are resolved via `Config` singleton which reads `config.json`:

```json
{
    "windowTitle": "My App",
    "OpenGL_major_version": 4,
    "OpenGL_minor_version": 5,
    "N_OPENAL_SOURCES": 32,
    "expected_number_of_gameobjects": 1200,
    "application_assets_dir": "/path/to/app/assets",
    "engine_assets_dir": "/path/to/EngineAssets"
}
```

`config.json` is generated at build time from `config.json.in` with CMake variable substitution.

**Expected runtime layout:**
```
Executable/
├── config.json
├── OpenAL32.dll  (Windows)
├── Assets/       (application assets)
└── EngineAssets/ (shaders, built-in models)
```

---

## Coding Conventions

- **Classes:** PascalCase (`TransformComponent`, `AudioSystem`)
- **Private members:** `m_` prefix, snake_case (`m_renderer`, `m_audioSystem`)
- **Static members:** `s_` prefix (`s_logger`)
- **Raw/smart pointer members:** `Ptr` suffix (`m_rigidBodyPtr`, `m_motionStatePtr`)
- **Pimpl:** `p_Impl` naming (`Window`, `Input`)
- **No exceptions** in engine core — error handling via assertions + logging
- **Assertions:** `MONA_ASSERT(cond, msg)` — triggers debug break on Windows, disabled in release
- **Logging macros:** `MONA_LOG_INFO`, `MONA_LOG_WARNING`, `MONA_LOG_ERROR` (spdlog-backed)
- **Language:** Code comments may be in Spanish (bilingual codebase)
- Header/implementation split: `.hpp` / `.cpp`; internal details in `Detail/` subdirectories

---

## Event System

```cpp
// Subscribe
auto handle = world.GetEventManager().Subscribe<Mona::StartCollisionEvent>(
    this, &MyClass::OnCollision);

// Unsubscribe
world.GetEventManager().Unsubscribe(handle);

// Publish
world.GetEventManager().Publish(Mona::StartCollisionEvent{...});
```

**Built-in event types:** `WindowResize`, `MouseScroll`, `GameObjectDestroyed`, `ApplicationEnd`, `DebugGUI`, `StartCollision`, `EndCollision`, `CustomUserEvent`.

---

## Optional ECS (EnTT)

Enable with `world.EnableECS()`. Access via `World::GetECSHandler()`. Uses `MonaECS` namespace. See `source/ECS/README.md` for full documentation.

---

## Testing

There is no automated test suite. Validation is done through the example applications in `examples/`:

- **HelloMonaEngine** — basic rendering, mesh, texture
- **Breakout** — 2D audio, physics, input
- **AnimationAudio** — skeletal animation, IK, 3D audio, raycasting

To run examples, build the project and execute the example binary with the corresponding `config.json` and assets in place.

---

## Third-Party Libraries

All are Git submodules under `thirdParty/`. Do not modify submodule contents directly.

| Library | Version | Purpose |
|---|---|---|
| GLFW | latest | Window & input |
| Glad | — | OpenGL loader |
| GLM | 0.9.9.8 | Math |
| Bullet Physics | 3 | Physics |
| OpenAL-Soft | latest | Audio |
| Assimp | latest | Model loading |
| ImGui | 1.78 | Debug GUI |
| spdlog | latest | Logging |
| STB Image | — | Texture loading |
| dr_wav | — | WAV loading |
| EnTT | latest | Optional ECS |
| nlohmann/json | latest | JSON config |
| Tracy | latest | Profiler (opt-in) |
| Eigen | latest | IK math |
| debug-draw | — | Debug geometry |
| whereami2cpp | — | Executable path |

---

## Platform Notes

**Windows (primary):**
- Compiler: MSVC 2022
- Generator: Visual Studio 17 2022 or Ninja
- OpenAL DLL must be next to the executable

**Linux:**
- Required packages: `xorg-dev`, `libglu1-mesa-dev`, `libx11-dev`
- Some systems require elevated permissions for OpenAL
- Use `linux-debug` or `linux-release` preset

**macOS:**
- Not officially tested but theoretically supported via GLFW/C++20

---

## CI/CD

GitHub Actions workflows are in `.github/workflows/`. They run CMake multi-platform builds automatically on push/PR.

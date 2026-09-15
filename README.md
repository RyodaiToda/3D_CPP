# 3D Physics Demo

raylibで表示する、C++製の簡易3D剛体物理デモです。
箱と球を重力で落下させ、衝突判定と接触ソルバの結果を確認できます。

## Features

- raylibによる3D描画
- 箱と球の剛体
- 重力、速度更新、姿勢更新
- AABBによるBroad phase
- 箱同士のSAT衝突判定
- 球同士、球と箱の接触判定
- 摩擦、反発、位置補正
- 剛体IDごとの描画色
- マウスによるカメラ回転とホイールズーム

## Directory structure

```text
.
|-- CMakeLists.txt
|-- test.cpp
|-- phys/
|   |-- body.h
|   |-- collision.cpp
|   |-- collision.h
|   |-- math3d.h
|   |-- world.cpp
|   `-- world.h
`-- build/                  # CMake生成物
```

## Requirements

- Windows
- Visual Studio Build Tools with C++ support
- CMake
- Git
- Internet connection for the first configuration (raylib 5.5 is fetched by CMake)

この環境では `cmake.exe` がVisual Studio Build Tools内にあるため、PATHに登録されていない場合があります。

## Build

PowerShellで、このREADMEがある `test` フォルダを開きます。

```powershell
$cmake = "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"

& $cmake -S . -B build -G "Visual Studio 18 2026"
& $cmake --build build --config Debug --target test
```

CMakeがPATHに登録されている場合は、次の短い形式でも実行できます。

```powershell
cmake -S . -B build -G "Visual Studio 18 2026"
cmake --build build --config Debug --target test
```

## Run

```powershell
.\build\Debug\test.exe
```

アプリケーション制御ポリシーで実行がブロックされる場合は、大学や組織の管理者に実行許可を確認してください。これはC++のコンパイルエラーではありません。

## Controls

| Input | Action |
|---|---|
| Left/right mouse button + drag | Rotate the camera |
| Mouse wheel | Zoom |
| `A` | Move the camera target left |
| `D` | Move the camera target right |
| Close window / `Esc` | Exit |

## Demo recording

プロジェクト固有の録画GIFはまだ含まれていません。GIFを追加する場合は、次の場所に保存してください。

```text
docs/demo.gif
```

その後、READMEのこの位置に次を追加できます。

```markdown
![3D physics demo](docs/demo.gif)
```

Windowsでは、画面録画ツールで実行中のウィンドウを録画し、GIFとして `docs/demo.gif` に保存してください。GitHubなどで表示する場合は、ファイルサイズを小さくするため解像度とフレームレートを下げるのがおすすめです。

## Physics update flow

```text
integrateForces
    -> broadphase
    -> narrowphase
    -> prepareConstraints
    -> warmStart
    -> solveVelocities
    -> integratePositions
    -> correctPositions
```

衝突形状の判定は `phys/collision.cpp`、剛体の時間発展と接触解決は `phys/world.cpp` にあります。

## Notes

- `phys` の `.cpp` ファイルは `CMakeLists.txt` の `test` ターゲットに追加されています。
- `.h` ファイルは各 `.cpp` から `#include` されるため、通常は実行ターゲットへの追加は不要です。
- C4819が表示される場合は、ソースファイルの文字コードに関する警告です。ビルド成功とは別の問題です。
- 物理挙動を変更する主な設定は `phys/world.h` の `velocityIterations`、`positionIterations`、`penetrationSlop`、`positionCorrection` です。

# Prerrequisitos y punto de entrada

## Frontera con el GPT de instalación

Este tutorial **no empieza instalando WSL ni ROS 2**. El estudiante debe llegar con WSL 2 y ROS 2 Jazzy funcionales.

Si no ha completado esa etapa, dirigirlo al GPT:

**Soporte Instalaciones Robótica**  
`https://chatgpt.com/g/g-696a59fbc5748191801b7fb3896b7925-soporte-instalaciones-robotica`

El GPT tutorial puede realizar comprobaciones mínimas, pero no debe duplicar una guía completa de instalación.

## Comprobación mínima

```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO
ros2 --help
```

Resultado esperado: `jazzy` y una respuesta válida de `ros2`.

Para confirmar el sistema:

```bash
lsb_release -a
```

El entorno validado fue Ubuntu 24.04 en WSL 2.

## Clonar el repositorio

```bash
mkdir -p ~/proyectos/gazebo-tutorial
cd ~/proyectos/gazebo-tutorial
git clone https://github.com/dryanguasr/gazebo-codex-mobile-manipulator.git
cd gazebo-codex-mobile-manipulator
```

Si ya existe, no volver a clonarlo encima. Inspeccionar primero:

```bash
pwd
git status
git log -1 --oneline
```

## Dependencias del ejemplo

```bash
sudo apt update
sudo apt install python3-rosdep ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
```

Inicializar `rosdep` solo si aún no se hizo:

```bash
sudo rosdep init
```

Si ya fue inicializado, no borrar configuración para repetir el comando.

Después:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`package.xml` declara las dependencias necesarias. No se necesita un entorno virtual Python.

## Criterio para continuar

El estudiante está listo si:

1. ROS 2 Jazzy responde;
2. el repositorio está clonado;
3. `rosdep install` termina sin dependencias críticas pendientes;
4. puede pasar al build del workspace.

## Problemas del GPT anterior

Derivar al GPT de instalación si el fallo ocurre antes de disponer de ROS 2 Jazzy funcional: WSL no instala/inicia, no existe `/opt/ros/jazzy`, nunca se configuró ROS o hay problemas básicos de Ubuntu/WSL.

## Problemas de este GPT

Este GPT sí cubre: `colcon build`, package/overlay, lanzamiento de Gazebo, controladores, `/clock`, odometría/TF, cámara/bridge, detector, tracker y experimento A/B.

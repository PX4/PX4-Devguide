# Instalación avanzada en Linux - Casos de Uso

## Usando Adaptadores de Programación JTAG

Los usuarios de Linux necesitan explícitamente permitir acceso al bus USB para los adaptadores de programación JTAG.

> **Nota** Para Archlinux: reemplace el grupo plugdev con uucp en los siguientes comandos

Ejecute un simple `ls` en modo `sudo` para asegurar que los comandos de acontinuación funcionan correctamente:

```sh
sudo ls
```

Con `sudo` garantizando temporalmente derechos, ejecute este comando:

```sh
cat > $HOME/rule.tmp <<_EOF
# Todos los dispositivos 3D Robotics (incluido PX4)
SUBSYSTEM=="usb", ATTR{idVendor}=="26AC", GROUP="plugdev"
# Dispositivos FTDI (y Black Magic Probe) 
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", GROUP="plugdev"
# Dispositivos Olimex
SUBSYSTEM=="usb",  ATTR{idVendor}=="15ba", GROUP="plugdev"
_EOF
sudo mv $HOME/rule.tmp /etc/udev/rules.d/10-px4.rules
sudo /etc/init.d/udev restart
```

Los usuarios deben ser agregados al grupo **plugdev**:

```sh
sudo usermod -a -G plugdev $USER
```
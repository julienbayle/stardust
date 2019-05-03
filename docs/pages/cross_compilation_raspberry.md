# Cross compilation du Raspberry

## Téléchargement des dépendance
```shell
$ sudo apt-get install bc build-essential gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```
## Partage des clés SSH 

Cette étape permet de ne plus avoir à entrer le mot de passe pour se connecter sur le robot
```shell
ssh-keygen -t rsa
ssh-copy-id <robot_user>@<robot_ip>
```

## Clone du sysroot Raspberry

### Depuis la clé USB
```shell
$ ./scripts/download-rpi-sysroot.sh -f /media/<user>/writable/
```

### Depuis le réseau
```shell
$ ./scripts/download-rpi-sysroot.sh -s <robot_user>@<robot_ip>
```

## Compilation
```shell
$ ./scripts/build-rpi.sh
```

##  Déploiement
        
```shell
deploy-rpi.sh "
        echo "  --remote-user=user         (default: ubuntu)"
        echo "  --remote-hostname=hostname (default: stardust)"
        echo "  --version=stardustX        (default: username_current_date)"
        echo "  --robot-name=rX            (default:r1)"
        echo "  --ros-restart              (default : no)"
        echo "  --install-on-startup       (default: no)"
        echo "  --build                    (default:no)" 
```

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

## Compilation et déploiement

```shell
$ ./scripts/deploy-rpi.sh <robot_user>@<robot_ip> <dossier>
```

# Cross compilation du Raspberry

## Téléchargement des dépendance
```shell
$ sudo apt-get install bc build-essential gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

## Clone du sysroot Raspberry

### Depuis la clé USB
```shell
$ ./scripts/download-rpi-sysroot.sh -f /media/<user>/writable/
```

### Depuis le réseau
```shell
$ ./scripts/download-rpi-sysroot.sh -s ubuntu@<ip>
```

## Compilation
```shell
$ ./scripts/build-rpi.sh
```

## Compilation et déploiement
```shell
$ ./scripts/deploy-rpi.sh ubuntu@<ip>
```
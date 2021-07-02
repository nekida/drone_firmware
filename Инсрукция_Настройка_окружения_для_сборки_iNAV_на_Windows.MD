1. Установить подсистему Ubuntu на Windows из магазина приложений
2. При первом запуске, скорее всего возникнет ошибка, что-то вроде "installing this may take a few minutes ..." с 16-м кодом ошибки. 
Это решается путем запуска Windows-приложения PowerShell от имени админа и прописывания в его консоли соедующей команды: Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux (терминал Ubuntu должен быть выключен)
3. Повторно запустить консоль Ubuntu и подождать некоторое время, код ошибки должен пропасть, надпись вроде "take a few minutes ..." - нет.
4. После задания юзером логина и пароля необходимо поставить инструментарий:
  ```
  sudo apt update

  sudo apt-get install git
  
  sudo apt-get install make
  
  sudo apt-get install cmake
  
  sudo apt-get install ruby
  
  sudo apt-get install gcc
  ```
  или
  ```
  sudo apt update && sudo apt upgrade
  
  sudo apt install git make ruby cmake gcc
  ```

5. Нужно поставить ARM компилятор:
```
sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi gdb-arm-none-eabi openocd
```
(первый обязательно)

6. Качаем репозиторий. Предварительно нужно смотировать диск C:
```
cd /mnt/c
```
Клонируем:
```
git clone https://github.com/iNavFlight/inav.git
```

Если ошибка:
```
Cloning into 'inav'...
error: chmod on /mnt/c/inav/.git/config.lock failed: Operation not permitted
fatal: could not set 'core.filemode' to 'false'
```

Покинь дерикторию и введи следующее:
```
sudo umount /mnt/c
sudo mount -t drvfs C: /mnt/c -o metadata
```
и вернись к шагу клонирования

7. Перейти в директорию inav и создать директорию build:
```
mkdir build
cd build
cmake ..
```
Это приведет к установке cmake и его элементов

8. Построение:

Перейти в папку build 
```
cd build
```
и сделать make для таргета
```
make MATEKF722
```

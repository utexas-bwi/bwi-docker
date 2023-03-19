# Using Podman

Podman can be used in place of Docker to run daemonless containers, which is helpful because host system user's do not need to be given root access to run containers.  Learn more about [Podman](https://podman.io).  Most of the commands you usually run with `docker ...` can be run with `podman ...`, so you can even alias docker to podman - ie exisiting packages that use docker commands will work.

A machine admin should follow this guide to allow non-root users to run and develop containers.

requirements:
[podman 3.4.4+](https://podman.io/getting-started/installation)
[podman-compose](https://phoenixnap.com/kb/podman-compose)
[nvidia-container-toolkit]()

### Install podman
```
sudo apt update
sudo apt install podman
```
Setup the common repositories
```
sudo cp /usr/share/doc/podman/examples/registries.conf  /etc/containers/registries.conf
```

### Install podman-compose
```
sudo apt install python3-pip
sudo -H pip3 install --upgrade pip
pip3 install podman-compose
```

### Setup NVIDIA GPU support

1. Create the directory for OCI container hooks:
    ```
    sudo mkdir -p /usr/share/containers/oci/hooks.d/
    ```
2. Create a hook for the NVIDIA container toolkit by adding the following contents to the file `/usr/share/containers/oci/hooks.d/oci-nvidia-hook.json`:
    ```
    {
      "version": "1.0.0",
      "hook": {
          "path": "/usr/bin/nvidia-container-toolkit",
          "args": ["nvidia-container-toolkit", "prestart"],
          "env": [
              "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
          ]
      },
      "when": {
          "always": true,
         "commands": [".*"]
      },
      "stages": ["prestart"]
    }
    ```
3. Set up the nvidia container toolkit ([Link to Nvidia Documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#id9)):
    ```
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add - \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    ```
4. Now, install the NVIDIA Container Toolkit, if its not already installed:
    ```
    sudo apt-get update \
      && sudo apt-get install -y nvidia-container-toolkit
    ```
5. Test the install by running `nvidia-smi` in a container:
    ```
    podman run --rm --security-opt=label=disable \
       --hooks-dir=/usr/share/containers/oci/hooks.d/ \
       nvidia/cuda:11.6.2-base-ubuntu20.04 nvidia-smi
     ```
6. If you get the following error:
    ```
    Error: OCI runtime error: error executing hook `/usr/bin/nvidia-container-toolkit` (exit code: 1)
    ```
    1. Open `/etc/nvidia-container-runtime/config.toml`:
        ```
        sudo vim /etc/nvidia-container-runtime/config.toml
        ```
    2. Edit the commented out line from:
        ```
        # no-cgroups = false
        ```
        to:
        ```
        no-cgroups = true
        ```
    
**Notes**

<!-- https://askubuntu.com/a/1443886 -->
1. If you're upgrade or install NVIDIA drivers v525 on Ubuntu 22.04, you might face issues with display (screen blacking). To resolve that, open tty and run the following

    ```
    cd /etc/X11 
    sudo rm xorg.conf

### Use podman-compose

Just like with docker-compose, build and run projects from inside a directory containing a `docker-compose.yml` file.
```
podman-compose --podman-build-args='--format docker' build
```

Give the non-root podman user in the container access to the host's volumes:
```
podman unshare chown <uid>:<gid> <dir>
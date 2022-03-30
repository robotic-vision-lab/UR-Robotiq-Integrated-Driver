# Sphinx Documentation Generator

## Instructions Summary

> :warning: If you are running on Linux, please be sure to read the [Docker post-installation steps for Linux](https://docs.docker.com/engine/install/linux-postinstall/)

```console
$ sudo apt-get install docker.io
$ [do post installation stuff, not applicable on Windows or Mac]
$ docker pull mqt0029/rvl-ur-robotiq-driver
    [...]
$ docker tag mqt0029/rvl-ur-robotiq-driver:latest rvl-ur-robotiq-driver:latest
$ git clone --recurse-submodule https://github.com/robotic-vision-lab/UR-Robotiq-Integrated-Driver.git
    [...]
$ cd UR-Robotiq-Integrated-Driver/docker
$ sh launch_docker_container.sh     # or windows_launch_docker_container.sh
```

You are now in the Docker container.

```console
> run_rosdep
    [...]
> rebuild_catkin
    [...]
> pip install sphinx myst-parser sphinx-rtd-theme
> cd src/rvl_ur_robotiq/sphinx-docgen/
    [make changes to documentation in source/pages/[file].md]
> make html     # or latexpdf
```

Back on the host machine, after the documentation has been built, run `sh update_documentation.sh` to update the documentation in main directory.

```console
$ sh [...]/UR-Robotiq-Integrated-Driver/catkin_ws/src/rvl_ur_robotiq/sphinx-docgen/update_documentation.sh

or

$ sh catkin_ws/src/rvl_ur_robotiq/sphinx-docgen/update_documentation.sh
```

The script is written such that where it is called from should not matter.

<!---
## Requirements

Since the documentation is ROS dependant, it must be built within the container after all ROS modules
are built and properly sourced. 

> :warning: Be sure to follow [Quickstart instructions](../../../../documentation/rvl_driver_documentation.pdf) to ensure you have the Docker container! You may skip section 1.3.1 and 1.3.2.

Initial setup includes installing [Sphinx](https://www.sphinx-doc.org/en/master/) and some of its plugins for Markdown support.

```console
$ pip install sphinx myst-parser sphinx-rtd-theme
```

This will allow HTML generation. LaTeX requires additional tools to be installed. `sudo` is omitted since containers
will run as root by default. Windows users may see some permission issues and must use an elevated (run as Admin) terminal.

```console
$ apt-get -y install texlive-latex-recommended texlive-latex-extra texlive-fonts-recommended latexmk
```

## Making Changes

The file `index.rst` as the name suggests, provides the top-level tree that links the rest of the pages. Simply append the relative path (e.g. `pages/new_file.md`) and rebuild the documentation using the instructions below. New pages and content can be placed in `source/pages/` as a Markdown (.md) file with some additional features enabled by [Markedly Structured Text](https://myst-parser.readthedocs.io/en/latest/sphinx/intro.html). Since the documentation is in Markdown, GitHub will render some of it, but to fully view all the nice formatting and visual guides use the HTML version for best results.

## Building the Documents

Run `make html` to generate HTML or `make latexpdf` to generate LaTeX PDF documentation. The necessary files or folders
will be generated in `build/html` or `build/latex` respectively.

> :warning: The script requires that this repository and the UR-Robotiq-Integrated-Driver are in the same parent directory.

After generation, run the script using `sh update_documentation.sh` **FROM THE HOST** to move and overwrite the files on the public release repository and primary folder of this repository.
--->

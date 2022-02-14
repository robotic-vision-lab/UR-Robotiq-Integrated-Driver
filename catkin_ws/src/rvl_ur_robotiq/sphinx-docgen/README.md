# Sphinx Documentation Generator

## Requirements

Since the documentation is ROS dependant, it must be build within the container after all ROS modules
are built and sourced properly.

Initial setup includes install [Sphinx](https://www.sphinx-doc.org/en/master/) and some of it's plugins for Markdown support.

```console
$ pip install -r requirements.txt
```

This will allow HTML generation. LaTeX requires additional tools to be installed. `sudo` is omitted since containers
will run as root by default. Windows user might see some permission issue and must use an elevated (run as Admin) terminal.

```console
$ apt-get -y install texlive-full

OR

$ apt-get -y install texlive-latex-recommended texlive-latex-extra texlive-fonts-recommended latexmk
```

There is also a script to copy the documentation files and folders directly onto the public release repository. Hence, you must
also clone the public release repo, but submodules init are not necessary.

```console
$ git clone --recurse-submodules git@github.com:robotic-vision-lab/UR-Robotiq-Integrated-Driver.git
```

## Making Changes

The file `index.rst` as the name suggested, provides the top-level tree that links the rest of the pages. Simply append the relative path (e.g. `pages/new_file.md`) and
rebuild the documentation using the instructions below.

Otherwise, new additional pages and content can be placed in `source/pages/` as a Markdown (.md) file with some additional features enabled by [Markedly Structured Text](https://myst-parser.readthedocs.io/en/latest/sphinx/intro.html). Since the documentation is in Markdown, GitHub will render some of it, but to fully view all the nice formatting and visual guides, use the HTML version for best result.

## Building the Docs

Simply run `make html` to generate HTML or `make latexpdf` to generate LaTeX PDF documentation. The necessary files or folders
will be generated in `build/html` or `build/latex` respectively.

> :warning: The script requires that this repository and the UR-Robotiq-Integrated-Driver are in the same parent directory.

After generation, run the script using `sh update_documentation.sh` **FROM THE HOST** to move and overwrite files on the public release repository and primary folder of this repository.

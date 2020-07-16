# inkscape-laser-cutter-extension

Inkscape extension to export G-code compatible with open source laser cutters/engravers. It currently supports the following target platforms:

* [Marlin](https://marlinfw.org/)
* [Smoothieware](http://smoothieware.org/) including [Cohesion3D](https://cohesion3d.com/) boards
* [GRBL](https://github.com/gnea/grbl)
* [LinuxCNC](https://linuxcnc.org/)

## Dependencies 

The extension uses the [Pillow](https://pypi.org/project/Pillow/) (fork of [PIL](http://www.pythonware.com/products/pil/)) library to process image files. The code imports the libary under the name `PIL` which should work for either one.

The extension of course also needs [Inkscape](https://inkscape.org/), though the version isn't critical. It was tested with v0.92 as well as slightly earlier versions (before the 90 to 96 DPI change). It remains to be seen if it will be compatible with v1.0...

## Installing

The `lasercutter.py` and `lasercutter.inx` files must be copied/moved into the per-user Inkscape extensions folder. The `.py` file is the actual script, and the `.inx` file is XML describing the extension to Inkscape (including the details of the user interface for it's popup dialog).

On Linux, the extensions directory would be under `~/.config/inkscape/extensions`, on Windows this would be under `%AppData%\Inkscape\extensions` or something similar. You can find out for sure by opening up Inkscape's Preferences and checking the System Info section:

![Inkscape Preferences System Info screenshot](https://i.imgur.com/5Os4tNw.png "system info extensions directory")
![Inkscape extensions directory under Linux screenshot](https://i.imgur.com/G27UJ7Z.png "extensions directory under Linux")

As mentioned in Inkscape's documentation: https://inkscape.org/gallery/=extension/

## Running

Once the extension files are properly installed and Inkscape restarted, there should be a new extension menu item:

[![Inkscape extension menu item](https://i.imgur.com/yoAZeutm.png "overlapping shapes")](https://i.imgur.com/yoAZeut.png)

Screenshots of the dialog for the extension:

[![offset shapes](https://i.imgur.com/cn9nU1Sm.png "offset shapes")](https://i.imgur.com/cn9nU1S.png)
[![combined shapes](https://i.imgur.com/En0KFkom.png "combined shapes")](https://i.imgur.com/En0KFko.png)
[![overlapping shapes](https://i.imgur.com/9iR0aLmm.png "overlapping shapes")](https://i.imgur.com/9iR0aLm.png)

If you are curious about the toolpaths generated, you can preview the `.g` G-code files using [CAMotics](https://camotics.org/).

To generate a debian archive:

- tag the release
- `python setup.py sdist`
- `mkdir debian && cd debian`
- `py2dsc ../dist/minimalKB-x.x.x.tar.gz`
- `cd deb_dist/minimalkb-x.x.x`
- `dpkg-buildpackage -rfakeroot -uc -us -b`

The new debian package will be available in `debian/deb_dist`

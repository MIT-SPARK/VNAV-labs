# tesse-interface

Provides a Python interface to a __TESSE__ Unity build, which is described in [tesse-core](github.mit.edu/TESS/tesse-core).


## Setup

This package is compatible with Python 3.x, as well as Python 2.7 to enable usage with ROS.

To use this interface, clone then setup the `tesse` package.

```bash
git clone git@github.mit.edu:TESS/tesse-interface.git
cd tesse-interface
pip install -r requirements.txt  # Dependencies
python setup.py develop
```

## Usage

__TESSE__ provides a network interface, which this package leverages. See [this notebook](notebooks/python-demonstration.ipynb) for example usage.


### Disclaimer

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

© 2020 Massachusetts Institute of Technology.

MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.

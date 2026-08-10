from setuptools import setup


package_name="wt901_driver"



setup(

    name=package_name,

    version="1.0.0",

    packages=[
        package_name
    ],

    install_requires=[
        "setuptools",
        "pyserial"
    ],

    entry_points={

        "console_scripts":[

            "wt901_node = wt901_driver.wt901_node:main"

        ]

    },

)
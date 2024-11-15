from setuptools import setup, find_packages

setup(
    name="myLib",  # The name of your package
    version="1.5",
    packages=find_packages(),  # Automatically find packages (module_a, module_b)
    install_requires=[],  # Add any dependencies here (e.g., ["numpy", "requests"])
)

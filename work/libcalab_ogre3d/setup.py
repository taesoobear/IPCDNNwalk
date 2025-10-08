from setuptools import setup, find_packages, dist, Extension
import sys
class BinaryDistribution(dist.Distribution):
    def has_ext_modules(self):
        return True

# 현재 파이썬 버전 문자열 만들기 (예: '3.12')
current_version = f"{sys.version_info.major}.{sys.version_info.minor}"
setup(
    name="libcalab_ogre3d",
    description="library for character animation",
    packages=find_packages(),
    #package_dir={"": "libcalab"},
    python_requires=f"=={current_version}.*",  
    include_package_data = True,
    ext_modules=[
        Extension("libcalab_ogre3d.libmainlib", sources=[]),  # 소스 없음
    ],
    package_data={
        "libcalab_ogre3d": ["*.so", "*.dylib", "*.dll", "work/*", "*.pyd","*.py", "Resource/*","media3/*"],  # wheel 안에 포함할 파일
    },
    distclass=BinaryDistribution, 
    zip_safe=False,
)



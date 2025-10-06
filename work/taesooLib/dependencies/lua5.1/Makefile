install_macosx:
	brew install libpng doxygen fltk
	cd lua-5.1.5;make macosx install
install_linux:
	sudo apt install libreadline-dev 
	cd lua-5.1.5;make linux;sudo make install
	cd lua-5.1.5/src;mkdir -p build;cd build;cmake ..; make
	sudo cp lua-5.1.5/src/build/liblua51.so /usr/local/lib/
	sudo apt install libfltk1.3-dev
	echo "export LD_LIBRARY_PATH=\"/lib:/usr/lib:/usr/local/lib\"" >> ~/.bashrc


CXX=c++
CXX_OPT=-std=c++11 -stdlib=libc++ -Wall -I/Users/smithj1/ekf/lib -I./ 
FILES=*.cpp
OUT_EXE=run_ekf

build: $(FILES)
	$(CXX) $(CXX_OPT) $(FILES) -o $(OUT_EXE)

clean:
	-rm -rf $(OUT_EXE)

rebuild: clean build

CXX=c++
CXX_OPT=-std=c++11 -stdlib=libc++ -I/Users/smithj1/Documents/Code/ekf/lib -I./ 
CXX_WARN=-Wall -Wno-deprecated-register -Wno-mismatched-tags 
CXX_LIB=-L/Users/smithj1/Documents/Code/ekf/lib -L./
CXX_INCLUDE=-I/Users/smithj1/Documents/Code/ekf/include -I./
FILES=*.cpp
OUT_EXE=run_ekf

build: $(FILES)
	$(CXX) $(CXX_OPT) $(CXX_WARN) $(CXX_LIB) $(CXX_INCLUDE) $(FILES) -o $(OUT_EXE)

clean:
	-rm -rf $(OUT_EXE)

rebuild: clean build

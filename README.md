# sdn-vanet
SDN over VANET

NEST on NS-3

GET ns-3 by

    ~$ wget https://www.nsnam.org/release/ns-allinone-3.25.tar.bz2
    ~$ tar xjf ns-allinone-3.25.tar.bz2


place this repository by

    ~$ mv ns-allinone-3.25/ns-3.25/ ./
    ~$ rm -r ns-allinone-3.25
    ~$ git clone https://github.com/CHLLHC/sdn-vanet.git temp
    ~$ mv temp/.git ns-3.25/.git
    ~$ rm -r temp
    ~$ cd ns-3.25
    ~/ns-3.25$ git checkout -- .

configure by

    CXXFLAGS="-Wall -g -std=c++11" ./waf -d debug --enable-examples --boost-includes=/usr/local/include --boost-libs=/usr/local/lib configure
    ~/ns-3.25$ CXXFLAGS="-Wall -g -std=c++11" ./waf -d debug --enable-examples configure
    
build by

    ~/ns-3.25$ ./waf
    
OR

    ~/ns-3.25$ ./waf -j4
      (For 4 threads computer)
    ~/ns-3.25$ ./waf -j8
      (For 8 threads computer)

run by

    ~/ns-3.25$ ./waf --run "SDN"

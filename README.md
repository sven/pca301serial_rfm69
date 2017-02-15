# pca301serial_rfm69 - RFM69 port of the pca301serial firmware

This sketch for Arduino Nano is a port of the pca301serial library to the RFM69
transmitter. I also used this as a learning experience on how transmitters work
so I create a separate RFM69 access module by reading through various
forum/blog entries and sources on the web. Thanks to all contributors who
shared their work.

With this sketch you can now use FHEM combined with an Arduino Nano and an
RFM69 transmitter to access PCA301 remote outlets. Maybe also openHAB works.

A very easy guide on how to connect the RFM69 to the Arduino Nano can be found
at
[Steigerbaletts Blog](https://steigerbalett.wordpress.com/2015/05/23/jeelink-clone-loten-und-mit-einer-firmware-flashen-fur-lacrosse-sensoren-in-fhem).


## Tested Platforms

  * Arduino Nano with RFM69HW - Set RFM69\_IS\_HW to true (default).
  * [JeeLink 868 (v3c)](https://forum.fhem.de/index.php/topic,65880.0.html) - Thanks to Dirk. Set RFM69\_IS\_HW to false.


## Links

  * [pca301serial](http://fhem.de/commandref.html#PCA301)
  * [FHEM](http://fhem.de)
  * [openHAB](http://www.openhab.org)
  * [LaCrosseGateway V1.x](https://wiki.fhem.de/wiki/LaCrosseGateway_V1.x)
  * [Steigerbaletts Blog](https://steigerbalett.wordpress.com/2015/05/23/jeelink-clone-loten-und-mit-einer-firmware-flashen-fur-lacrosse-sensoren-in-fhem)


## Motivation

I own a Brother DCP-135c printer that is now running for years and ink
cartridges are very cheap. Also it has a good but not perfect driver for Linux.
A big problem with this printer is that it decides by himself when to clean the
print heads. It also doesn't have a clock so you can't tell it to only clean at
daytime. Another problem with the permanent cleaning is that it empties out the
cartridges. So since around 2008
[Penny Market sells Multifunctionprinters (german)](http://www.pcwelt.de/news/Discounter-Penny-verkauft-Multifunktionsgeraet-fuer-57-Euro-225995.html)
the printer is only switched on when it's needed. It turned out that the
cleaning isn't really necessary because it's used regularly.

This worked well as long as the printer stood beside the PC that prints. But
then something changed and now the printer moved downstairs. This turned down
the
[WAF - Wife Acceptance Factor](https://en.wikipedia.org/wiki/Wife_acceptance_factor)
a bit so I searched for a solution. First thing that came to my mind was a
cheap remote controlled outlet set from some supermarket controlled by the
printing service on the remote PC that connects to the printer. The problem
with this cheap solution was that there is no backchannel to verfiy if the
remote signal really reached the outlet. Other solutions did include LAN
outlets and for example AVM outlet solutions. But after some googling I found
FHEM and reports that the PCA301 was cheap and had a backchannel. As a bonus it
also measures the power and doesn't pollute the air with periodic status
transfers.

Also I got infected by the Home Automation fever a bit and this should become
my first step.

The only problem and the reason for this port of the pca301serial library was
that I thought that RFM69 would work with the Arduino Nano. But after ordering
two and also two RFM69s I found out that the RFM69 is only supported for other
devices _or_ that PCA301 needs a wifi ESP chip instead of the Arduino Nano.


## Costs

  * Arduino Nano, eBay: 2.99 €
  * RFM69HW, 868 MHz, eBay: 2.45 €


## MIT License

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

---


---

<hr>
<p>layout: post</p>
<p>title: “curbdetection”</p>
<p>excerpt: “A ton of text to test readability.”</p>
<p>categories: [paragraph]</p>
<p>comments: true</p>
<hr>
<h1 id="method1-baoxing">method1: BaoXing</h1>
<p>ref: Curb-Intersection Feature Based Monte Carlo Localization on Urban Roads</p>
<ul>
<li>segmentation of laser scan</li>
</ul>
<p><img src="https://cdn.nlark.com/yuque/0/2018/png/134562/1542274808431-2bdf085b-12c1-4e0e-b3ec-9e4f28d7f531.png" alt="image.png | center | 647x450"></p>
<p>ss</p>
<ol>
<li>piecewise function of laserscan</li>
</ol>
<p><img src="https://cdn.nlark.com/yuque/0/2018/png/134562/1542280692169-d953a089-8fe6-48a7-b2cc-57c712b37c6b.png" alt="image.png | center | 544x172"></p>
<ol start="2">
<li>use second-order differential filter to get local minimum-maximum detection point</li>
</ol>
<p><img src="https://cdn.nlark.com/yuque/0/2018/png/134562/1542280807987-af331f62-7e8d-415f-8ef0-a33f889f20cf.png" alt="image.png | center | 582x200"></p>
<p>we can think this as, after discussion woth Peng, we believe it’s one-order differential function(the same in the picture, red curve):</p>
<div id="9qo3nz"><img src="https://cdn.nlark.com/__latex/af63c714c1f6b7b1e9b964cef99fdf52.svg" width="138"></div>
* classification of the scan
1. Road surface segment, shown as line CD, is selected first. It always locates between two edgepoints nearest to center of the sensor.
2. Curb lines, (BC and DE), are searchedsubsequently, based on point C and D determined fromthe former step.
3. Rest segments are other features off the road.
* monte-carlo localization with these features
1. prediction with odom(easy part)
2. correction with two kind of features
* curb point
* intersection point
<p><img src="https://cdn.nlark.com/yuque/0/2018/png/134562/1542281566591-1dcf8a60-fe29-45b3-a889-89541057a48b.png" alt="image.png | center | 626x324"></p>
<ol start="3">
<li>resampling</li>
</ol>
<ul>
<li>curb-intersection measurement model</li>
</ul>
<ol>
<li>LIDAR-VSA1<br>
accumulate these curb point, and translate them to last coordinate</li>
<li>LIDAR-VSA2<br>
it’s just two parallel point, tagent to CD.And whenever at intersection, we get two these points</li>
</ol>
<h1 id="method2">method2</h1>


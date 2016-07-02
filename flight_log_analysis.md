# Flight Log Analysis

There are several software packages that exist to analyze PX4 flight logs. They are described below.

## Log Muncher

### Upload

Users can visit this webpage and upload log files directly: [http://logs.uaventure.com/](http://logs.uaventure.com/)

![](images/flight_log_analysis/logmuncher.png)

### Result

![](images/flight_log_analysis/log-muncher-result.png)

[Example Log](http://logs.uaventure.com/view/KwTFDaheRueMNmFRJQ3huH)

### Postive
*  web based, great for end-users
*  user can upload load and then share report with others

### Negative
* analysis is very constrained, no customization possible

## Flight Plot

![](https://pixhawk.org/_media/dev/flightplot-0.2.16-screenshot.png)

### Positive
* java based, cross-platform
* intuitive GUI, no programming knowledge required

### Negative
* java libraries brittle
* analysis constrained by what features have been built-in

## [PX4Tools](https://github.com/dronecrew/px4tools)

![](images/flight_log_analysis/px4tools.png)

### Install

* The recommended procedure is to use anaconda3. See [px4tools github page](https://github.com/dronecrew/px4tools) for details.

```bash
conda install -c https://conda.anaconda.org/dronecrew px4tools
```

### Positive
* python based, cross platform, works witn anaconda 2 and anaconda3
* ipython/ jupyter notebooks can be used to share analysis easily
* advanced plotting capabilities to allow detailed analysis

### Negative
* Requires the user to know python
* Currently requires use of sdlog2_dump.py or px4tools embedded px42csv program to convert log files to csv before use
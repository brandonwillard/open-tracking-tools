This library contains a set of tools for tracking objects through GPS data real-time.  
Specifically, it contains implementations of Bayesian particle filters for on-road and off-road 2D 
motion and state-parameter estimation.

The generic api uses a graph and some observations to produce sequential filtered results, which infer the
true location, velocity, their variances, the street it's on/the path it took.  Extensions in this library include
the ability to estimate parameters, like GPS and acceleration errors.
See https://github.com/openplans/open-tracking-tools/wiki for a better description.

To get started, you can build an OpenTripPlanner graph by specifying coordinate boundaries for OpenStreetMap data, then
running the TraceRunner in open-tracking-tools-otp over a CSV file.  (See https://github.com/openplans/OpenTripPlanner/wiki/GraphBuilder for more
information about OTP graph building.)  An example xml build file for an OTP graph can be found in the
open-tracking-tools-otp project, as well as an example TraceRunner config file.

Another example of a graph is GenericJTSGraph.java, which takes simple JTS geometry objects.

There is also a simulator that will produce observations for testing.  See RoadTrackingGraphFilterTest.java for
a complete example of graph construction, simulation and filtering. 


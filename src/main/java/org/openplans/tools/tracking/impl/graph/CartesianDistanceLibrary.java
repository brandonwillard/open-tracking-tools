package org.openplans.tools.tracking.impl.graph;

import org.opentripplanner.common.geometry.DistanceLibrary;

import com.vividsolutions.jts.geom.Coordinate;

public class CartesianDistanceLibrary implements DistanceLibrary {

  @Override
  public double distance(Coordinate from, Coordinate to) {
    final double xd = from.x - to.x;
    final double yd = from.y - to.y;
    return Math.sqrt(xd * xd + yd * yd);
  }

  @Override
  public double distance(double lat1, double lon1, double lat2,
    double lon2) {
    final double xd = lon1 - lon2;
    final double yd = lat1 - lat2;
    return Math.sqrt(xd * xd + yd * yd);
  }

  @Override
  public double distance(double lat1, double lon1, double lat2,
    double lon2, double radius) {
    final double xd = lon1 - lon2;
    final double yd = lat1 - lat2;
    return Math.sqrt(xd * xd + yd * yd);
  }

  @Override
  public double fastDistance(Coordinate from, Coordinate to) {
    final double xd = from.x - to.x;
    final double yd = from.y - to.y;
    return Math.sqrt(xd * xd + yd * yd);
  }

  @Override
  public double fastDistance(double lat1, double lon1, double lat2,
    double lon2) {
    final double xd = lon1 - lon2;
    final double yd = lat1 - lat2;
    return Math.sqrt(xd * xd + yd * yd);
  }

}

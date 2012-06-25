package org.openplans.tools.tracking.impl.graph;

import org.opentripplanner.common.geometry.DistanceLibrary;

import com.vividsolutions.jts.geom.Coordinate;

public class CartesianDistanceLibrary implements DistanceLibrary {

	@Override
	public double distance(Coordinate from, Coordinate to) {
		double xd = from.x - to.x;
		double yd = from.y - to.y;
		return Math.sqrt(xd * xd + yd * yd); 
	}

	@Override
	public double fastDistance(Coordinate from, Coordinate to) {
		double xd = from.x - to.x;
		double yd = from.y - to.y;
		return Math.sqrt(xd * xd + yd * yd);
	}

	@Override
	public double distance(double lat1, double lon1, double lat2, double lon2) {
		double xd = lon1 - lon2;
		double yd = lat1 - lat2;
		return Math.sqrt(xd * xd + yd * yd);
	}

	@Override
	public double fastDistance(double lat1, double lon1, double lat2,
			double lon2) {
		double xd = lon1 - lon2;
		double yd = lat1 - lat2;
		return Math.sqrt(xd * xd + yd * yd);
	}

	@Override
	public double distance(double lat1, double lon1, double lat2, double lon2,
			double radius) {
		double xd = lon1 - lon2;
		double yd = lat1 - lat2;
		return Math.sqrt(xd * xd + yd * yd);
	}

}

package org.opentrackingtools.graph;

import com.google.common.collect.ComparisonChain;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.linearref.LinearLocation;

public class InferenceGraphSegment implements Comparable<InferenceGraphSegment> {

  final protected LineSegment line;
  final protected InferenceGraphEdge parentEdge;
  final protected LinearLocation startIndex;
  final protected LinearLocation endIndex;
  final protected double startDistance;
  
  public InferenceGraphSegment(LineSegment line, InferenceGraphEdge infEdge) {
    this.line = line;
    this.parentEdge = infEdge;
    this.startIndex = infEdge.getLocationIndexedLine().indexOf(line.p0);
    this.endIndex = infEdge.getLocationIndexedLine().indexOfAfter(line.p1, this.startIndex);
    this.startDistance = infEdge.getLengthLocationMap().getLength(this.startIndex);
  }

  public double getStartDistance() {
    return startDistance;
  }

  public LineSegment getLine() {
    return line;
  }

  public InferenceGraphEdge getParentEdge() {
    return parentEdge;
  }

  public LinearLocation getStartIndex() {
    return startIndex;
  }

  public LinearLocation getEndIndex() {
    return endIndex;
  }

  @Override
  public int compareTo(InferenceGraphSegment o) {
    return ComparisonChain
        .start()
        .compare(this.startDistance, o.startDistance)
        .compare(this.parentEdge, o.parentEdge)
        .result();
  }

}

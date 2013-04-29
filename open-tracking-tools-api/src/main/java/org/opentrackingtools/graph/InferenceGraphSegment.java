package org.opentrackingtools.graph;

import com.google.common.collect.ComparisonChain;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.linearref.LinearLocation;

import org.apache.commons.lang3.builder.ToStringBuilder;

public class InferenceGraphSegment implements Comparable<InferenceGraphSegment> {

  final protected LineSegment line;
  final protected InferenceGraphEdge parentEdge;
  final protected LinearLocation startIndex;
  final protected LinearLocation endIndex;
  final protected double startDistance;
  final protected InferenceGraphSegment nextSegment;
  
  public InferenceGraphSegment(LineSegment line, InferenceGraphEdge infEdge, InferenceGraphSegment nextSegment) {
    this.line = line;
    this.parentEdge = infEdge;
    this.startIndex = infEdge.getLocationIndexedLine().indexOf(line.p0);
    this.endIndex = infEdge.getLocationIndexedLine().indexOfAfter(line.p1, this.startIndex);
    this.startDistance = infEdge.getLengthLocationMap().getLength(this.startIndex);
    this.nextSegment = nextSegment;
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

  public InferenceGraphSegment getNextSegment() {
    return this.nextSegment;
  }

  @Override
  public String toString() {
    ToStringBuilder builder = new ToStringBuilder(this);
    builder.append("line", line);
    builder.append("parentEdge", parentEdge.getEdgeId());
    builder.append("startIndex", startIndex.getSegmentIndex());
    return builder.toString();
  }

}

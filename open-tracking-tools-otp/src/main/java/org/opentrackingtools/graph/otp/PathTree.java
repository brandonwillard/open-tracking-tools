package org.opentrackingtools.graph.otp;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;

public class PathTree {
  HashMap<PathEdge, PathTree> children =
      new HashMap<PathEdge, PathTree>();
  boolean isLeaf = false;
  PathTree parent;
  List<Path> paths = new ArrayList<Path>();

  public PathTree() {
  }

  public PathTree(PathTree parent) {
    this.parent = parent;
  }

  public PathTree apply(PathEdge edge, Path path) {
    PathTree next = this.children.get(edge);

    if (next == null) {
      next = new PathTree(this);
      this.children.put(edge, next);
      this.paths.add(path);
      return next;
    } else {
      //do not yet add path to the list of paths for the
      //this because we might need to remove those paths
      return next;
    }
  }

  public PathTree get(PathEdge edge) {
    return this.children.get(edge);
  }

  boolean isLeaf() {
    return this.isLeaf;
  }

  public void removePath(Path path) {
    PathTree tree = this;
    while (tree != null) {
      tree.paths.remove(path);
      tree = tree.parent;
    }
  }
}

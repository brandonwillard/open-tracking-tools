package org.openplans.tools.tracking.impl.statistics;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import com.google.common.collect.Lists;

public class DataCube implements Serializable {

  // number of minutes per interval
  public static Integer INTERVAL = 60;

  private final List<Double> data = Lists.newArrayList();

  private final ConcurrentHashMap<String, ConcurrentHashMap<Integer, ConcurrentHashMap<Integer, Boolean>>> attributeMap =
      new ConcurrentHashMap<String, ConcurrentHashMap<Integer, ConcurrentHashMap<Integer, Boolean>>>();

  // fillter by attributes and group
  public HashMap<Integer, Double> filterAndGroup(
    HashMap<String, Integer> filterAttributes, String group) {
    final HashMap<Integer, Double> resultSet =
        new HashMap<Integer, Double>();

    Set<Integer> filterSet = null;

    // create filter set
    for (final String attribute : filterAttributes.keySet()) {
      if (attributeMap.containsKey(attribute)) {
        if (attributeMap.get(attribute).contains(
            filterAttributes.get(attribute))) {
          if (filterSet == null)
            filterSet =
                attributeMap.get(attribute)
                    .get(filterAttributes.get(attribute)).keySet();
          else
            filterSet.retainAll(attributeMap.get(attribute)
                .get(filterAttributes.get(attribute)).keySet());
        }
      }
    }

    // group values and filter
    if (attributeMap.containsKey(group)) {
      for (final Integer g : attributeMap.get(group).keySet()) {
        final Set<Integer> dataSet =
            attributeMap.get(group).get(g).keySet();

        if (filterSet != null)
          dataSet.retainAll(filterSet);

        // hardcoding mean as aggregating function -- need to allow for other approaches
        Double sum = 0.0;
        for (final Integer i : dataSet)
          sum += data.get(i);

        resultSet.put(g, new Double(sum / dataSet.size()));
      }
    }

    return resultSet;
  }

  // store d  in the cube with the given map of attributes
  public void store(Double d, HashMap<String, Integer> attributes) {
    // store data point 
    final Integer dataId = data.size();
    data.add(d);

    // store data point reference in attributeMap for attributes

    for (final String attribute : attributes.keySet()) {
      if (!attributeMap.containsKey(attribute))
        attributeMap
            .put(
                attribute,
                new ConcurrentHashMap<Integer, ConcurrentHashMap<Integer, Boolean>>());
      final Integer value = attributes.get(attribute);

      if (!attributeMap.get(attribute).containsKey(value))
        attributeMap.get(attribute).put(value,
            new ConcurrentHashMap<Integer, Boolean>());

      attributeMap.get(attribute).get(value)
          .put(dataId, Boolean.TRUE);
    }
  }

  static public DataCube read(File inFile) {
    DataCube dc = new DataCube();

    try {
      final FileInputStream fileInStream =
          new FileInputStream(inFile);
      final ObjectInputStream objInStream =
          new ObjectInputStream(fileInStream);

      try {
        dc = (DataCube) objInStream.readObject();
      } catch (final ClassNotFoundException cnfException) {
        // TODO need to handle CNF exception 
      }

      objInStream.close();
    } catch (final IOException ioException) {
      // TODO need to handle io exception 
    }

    return dc;
  }

  static public void write(DataCube dc, File outFile) {
    try {
      final FileOutputStream fileOutStream =
          new FileOutputStream(outFile);
      final ObjectOutputStream objOutStream =
          new ObjectOutputStream(fileOutStream);

      objOutStream.writeObject(dc);

      objOutStream.flush();
      objOutStream.close();
    } catch (final IOException ioException) {
      // TODO need to handle io exception
    }
  }
}

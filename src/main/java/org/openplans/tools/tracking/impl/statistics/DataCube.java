package org.openplans.tools.tracking.impl.statistics;

import gov.sandia.cognition.statistics.distribution.NormalInverseGammaDistribution;
import gov.sandia.cognition.math.matrix.Vector;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;

public class DataCube implements Serializable {
	
	// number of minutes per interval
	public static Integer INTERVAL = 60;
	
	private List<Double> data = Lists.newArrayList();
	
	private ConcurrentHashMap<String, ConcurrentHashMap<Integer, ConcurrentHashMap<Integer, Boolean>>> attributeMap = new ConcurrentHashMap<String, ConcurrentHashMap<Integer, ConcurrentHashMap<Integer, Boolean>>>();

	static public void write(DataCube dc, File outFile)
	{
		try
		{
			FileOutputStream fileOutStream = new FileOutputStream(outFile);
			ObjectOutputStream objOutStream = new ObjectOutputStream(fileOutStream);
			
			objOutStream.writeObject(dc);
			
			objOutStream.flush();
			objOutStream.close();
		}
		catch(IOException ioException)
		{
			// TODO need to handle io exception
		}
	}
	  
	static public DataCube read(File inFile)
	{
		DataCube dc = new DataCube();
	
		try
		{
			FileInputStream fileInStream = new FileInputStream(inFile);
			ObjectInputStream objInStream = new ObjectInputStream(fileInStream);
			  
			try
			{
				dc = (DataCube)objInStream.readObject();
			}
			catch(ClassNotFoundException cnfException)
			{
				// TODO need to handle CNF exception 
			}
			
			objInStream.close();
		}
		catch(IOException ioException)
		{
			// TODO need to handle io exception 
		}
		   
		return dc;
	}
	
	
	// store d  in the cube with the given map of attributes
	public void store(Double d, HashMap<String, Integer> attributes)
	{
		// store data point 
		Integer dataId = data.size();
		data.add(d);
		
		// store data point reference in attributeMap for attributes
		
		for(String attribute : attributes.keySet())
		{
			if(!attributeMap.containsKey(attribute))
				attributeMap.put(attribute, new ConcurrentHashMap<Integer, ConcurrentHashMap<Integer, Boolean>>());
			Integer value = attributes.get(attribute);
			
			if(!attributeMap.get(attribute).containsKey(value))
				attributeMap.get(attribute).put(value, new ConcurrentHashMap<Integer, Boolean>());
			
			attributeMap.get(attribute).get(value).put(dataId, Boolean.TRUE);
		} 
	}	
	
	// fillter by attributes and group
	public HashMap<Integer, Double> filterAndGroup(HashMap<String, Integer> filterAttributes, String group)
	{
		HashMap<Integer, Double> resultSet = new HashMap<Integer, Double>();
		
		Set<Integer> filterSet = null;
		
		// create filter set
		for(String attribute : filterAttributes.keySet())
		{
			if(attributeMap.containsKey(attribute))
			{
				if(attributeMap.get(attribute).contains(filterAttributes.get(attribute)))
				{
					if(filterSet == null)
						filterSet = attributeMap.get(attribute).get(filterAttributes.get(attribute)).keySet();
					else
					    filterSet.retainAll(attributeMap.get(attribute).get(filterAttributes.get(attribute)).keySet());
				}
			}
		}
		
		
		// group values and filter
		if(attributeMap.containsKey(group))
		{
			for(Integer g : attributeMap.get(group).keySet())
			{
				Set<Integer> dataSet = attributeMap.get(group).get(g).keySet();
				
				if(filterSet != null)
					dataSet.retainAll(filterSet);
				
				// hardcoding mean as aggregating function -- need to allow for other approaches
				Double sum = 0.0;
				for(Integer i : dataSet)
					sum += data.get(i);
				
				resultSet.put(g, new Double(sum / dataSet.size()));
			}
		}
		
		return resultSet;
	}
}
		
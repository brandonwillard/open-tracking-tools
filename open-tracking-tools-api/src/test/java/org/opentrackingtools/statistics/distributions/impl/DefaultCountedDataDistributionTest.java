package org.opentrackingtools.statistics.distributions.impl;

import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;

import org.testng.AssertJUnit;
import org.testng.annotations.Test;

import java.util.Random;

public class DefaultCountedDataDistributionTest {

  @Test
  public void testDefaultCountedDataDistributionLogScale() {
    DefaultCountedDataDistribution<String> testDist = new DefaultCountedDataDistribution<String>(true);
    AssertJUnit.assertTrue(testDist.isLogScale());
    AssertJUnit.assertTrue(testDist.isEmpty());
    
    testDist.set("item1", Math.log(10d), 1);
    testDist.set("item2", Math.log(20d), 1);
    testDist.set("item3", Math.log(30d), 1);
    testDist.set("item4", Math.log(40d), 1);
    
    AssertJUnit.assertEquals(1, testDist.getCount("item1"), 1e-2);
    AssertJUnit.assertEquals(4, testDist.getTotalCount());
    AssertJUnit.assertEquals(Math.log(10d), testDist.get("item1"), 1e-2);
    AssertJUnit.assertEquals(Math.log(100d), testDist.getTotal(), 1e-2);
    AssertJUnit.assertEquals(-Math.log(10d), testDist.getLogFraction("item1"), 1e-2);
    AssertJUnit.assertEquals(1d/10d, testDist.getFraction("item1"), 1e-2);
    AssertJUnit.assertEquals(testDist.getFraction("item1"), Math.exp(testDist.getLogFraction("item1")), 1e-2);
    
    /*
     * 'set' should simply reset this value
     */
    testDist.set("item1", Math.log(20d), 1);
    testDist.set("item2", Math.log(10d), 1);
    
    AssertJUnit.assertEquals(4, testDist.getTotalCount());
    AssertJUnit.assertEquals(Math.log(100d), testDist.getTotal(), 1e-2);
    AssertJUnit.assertEquals(Math.log(2d) - Math.log(10d), testDist.getLogFraction("item1"), 1e-2);
    AssertJUnit.assertEquals(2d/10d, testDist.getFraction("item1"), 1e-2);
    AssertJUnit.assertEquals(testDist.getFraction("item1"), Math.exp(testDist.getLogFraction("item1")), 1e-2);
    
    
    testDist.set("item1", Math.log(10), 2);
    testDist.set("item2", Math.log(20), 4);
    AssertJUnit.assertEquals(8, testDist.getTotalCount());
    AssertJUnit.assertEquals(Math.log(100d), testDist.getTotal(), 1e-2);
    AssertJUnit.assertEquals(4, testDist.getDomainSize());
    
    DefaultCountedDataDistribution<String> testDistClone = testDist.clone();
    
    AssertJUnit.assertEquals(testDist.getDomainSize(), testDistClone.getDomainSize());
    AssertJUnit.assertEquals(testDist.getTotal(), testDistClone.getTotal());
    AssertJUnit.assertEquals(testDist.getTotalCount(), testDistClone.getTotalCount());
    AssertJUnit.assertEquals(testDist.getLogFraction("item1"), testDistClone.getLogFraction("item1"));
    AssertJUnit.assertEquals(testDist.getLogFraction("item2"), testDistClone.getLogFraction("item2"));
    AssertJUnit.assertEquals(testDist.getLogFraction("item3"), testDistClone.getLogFraction("item3"));
    AssertJUnit.assertEquals(testDist.getLogFraction("item4"), testDistClone.getLogFraction("item4"));
    AssertJUnit.assertEquals(testDist.getCount("item1"), testDistClone.getCount("item1"));
    AssertJUnit.assertEquals(testDist.getCount("item2"), testDistClone.getCount("item2"));
    AssertJUnit.assertEquals(testDist.getCount("item3"), testDistClone.getCount("item3"));
    AssertJUnit.assertEquals(testDist.getCount("item4"), testDistClone.getCount("item4"));
    
    testDist.increment("item1", Math.log(10));
    AssertJUnit.assertEquals(9, testDist.getTotalCount());
    AssertJUnit.assertEquals(Math.log(110d), testDist.getTotal(), 1e-2);
    
    testDist.increment("item1", Math.log(10), 2);
    AssertJUnit.assertEquals(11, testDist.getTotalCount());
    AssertJUnit.assertEquals(Math.log(120d), testDist.getTotal(), 1e-2);
    
    /*
     * Can't test negative increments; doesn't make sense here.
     * 'decrement' should be used instead.
     */
//    testDist.increment("item1", -Math.log(10));
//    AssertJUnit.assertEquals(8, testDist.getTotalCount());
//    AssertJUnit.assertEquals(Math.log(100d), testDist.getTotal(), 1e-2);
    
    Random rng = new Random(1234533l);
    final int numSamples = (int) 1e5;
    DefaultCountedDataDistribution<String> sampleDist = new DefaultCountedDataDistribution<String>(true);
    for (String val : testDist.sample(rng, numSamples)) {
      sampleDist.increment(val);
    }
    
    AssertJUnit.assertEquals(numSamples, sampleDist.getTotalCount());
    AssertJUnit.assertEquals(testDist.getLogFraction("item1"), sampleDist.getLogFraction("item1"),
        1e-2);
//         Math.sqrt(testDist.getFraction("item1") * (1d - testDist.getFraction("item1"))));
    AssertJUnit.assertEquals(testDist.getLogFraction("item2"), sampleDist.getLogFraction("item2"), 
        1e-2);
//         Math.sqrt(testDist.getFraction("item2") * (1d - testDist.getFraction("item2"))));
    AssertJUnit.assertEquals(testDist.getLogFraction("item3"), sampleDist.getLogFraction("item3"),
        1e-2);
//         Math.sqrt(testDist.getFraction("item3") * (1d - testDist.getFraction("item3"))));
    AssertJUnit.assertEquals(testDist.getLogFraction("item4"), sampleDist.getLogFraction("item4"),
        1e-2);
//         Math.sqrt(testDist.getFraction("item4") * (1d - testDist.getFraction("item4"))));
    
  }
}

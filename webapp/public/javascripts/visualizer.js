/* This program is free software: you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public License
   as published by the Free Software Foundation, either version 3 of
   the License, or (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 */

var vehicleId;
var startLatLng;
$.ajax({
  url : '/api/getGraphCenter',
  dataType : 'json',
  async : false,
  cache : true,
  success : function(data) {
    startLatLng = new L.LatLng(data.lat, data.lng);
  }
});
var map;

var vertexLayer = null, edgeLayer = null;

//var cloudmadeUrl = 'http://{s}.tiles.mapbox.com/v3/mapbox.mapbox-streets/{z}/{x}/{y}.png';
var cloudmadeUrl = 'http://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';
var cloudmadeAttrib = 'Map data &copy; 2011 OpenStreetMap contributors, Imagery &copy; 2011 CloudMade';
var cloudmadeOptions = {
  maxZoom : 19,
  attribution : cloudmadeAttrib
};

var pointsGroup = new L.LayerGroup();
var edgeGroup = new L.LayerGroup();
var inferredGroup = new L.LayerGroup();
var actualGroup = new L.LayerGroup();
var evaluatedGroup = new L.LayerGroup();
var addedGroup = new L.LayerGroup();
var allInfEdgesGroup = new L.LayerGroup();
var allInfMeansGroup = new L.LayerGroup();

var lines = new Array();

// FIXME hack!
var globalEpsgCode = null;

var interval = null;
var paths = null;

var MAX_SPEED = 10; // in m/s

Ext.require([
    'Ext.direct.*',
    'Ext.data.*',
    'Ext.grid.*',
    'Ext.tab.*',
    'Ext.form.*',
    'Ext.ux.*',
    'Ext.selection.*',
    'Ext.layout.container.Border'
]);

Ext.define('Particle', {
  extend: 'Ext.data.Model',
  fields: [
           {name: 'number'},
           {name: 'weight'},
           {name: 'count'},
           {name: 'state'},
           {name: 'edge'},
           {name: 'distMoved'},
           {name: 'path'},
           {name: 'edgeMarginalLogLik'},
           {name: 'edgeTransLogLik'},
           {name: 'measurementLogLik'},
           {name: 'currentStateSample'},
           {name: 'prevStateSample'},
           {name: 'obsCovPrior'},
           {name: 'stateCovPrior'},
           {name: 'offTransPrior'},
           {name: 'onTransPrior'}
           ]
});

var postParticleStore = Ext.create('Ext.data.Store', {
  model: 'Particle',
  proxy: {
    type: 'memory',
    reader: {
        type: 'json',
    }
  }
});

var priorParticleStore = Ext.create('Ext.data.Store', {
  model: 'Particle',
  proxy: {
    type: 'memory',
    reader: {
        type: 'json',
    }
  }
});

Ext.define('Path', {
  extend: 'Ext.data.Model',
  fields: [
           {name: 'id'},
           {name: 'length'},
           {name: 'edges'},
           {name: 'data'}
           ]
});

var pathsStore = Ext.create('Ext.data.Store', {
  model: 'Path',
  proxy: {
    type: 'memory',
    reader: {
        type: 'json',
    }
  }
});

var gridColumns = [
        {
            text: 'number',
            flex:1,
            sortable: true,
            dataIndex: 'number'
        },
        {
            text: 'weight',
            flex:1,
            sortable: true,
            dataIndex: 'weight'
        },
        {
            text: 'count',
            flex:1,
            sortable: true,
            dataIndex: 'count'
        },
        {
            text: 'state',
            flex:1,
            sortable: false,
            dataIndex: 'state'
        },
        {
            text: 'edge',
            flex:1,
            sortable: true,
            dataIndex: 'edge'
        },
        {
            text: 'distMoved',
            flex:1,
            sortable: true,
            dataIndex: 'distMoved'
        },
        {
            text: 'path',
            flex:1,
            sortable: true,
            dataIndex: 'path'
        },
        {
            text: 'currentStateSample',
            flex:1,
            sortable: true,
            dataIndex: 'currentStateSample'
        },
        {
            text: 'prevStateSample',
            flex:1,
            sortable: true,
            dataIndex: 'prevStateSample'
        },
        {
            text: 'obsCovPrior',
            flex:1,
            sortable: true,
            dataIndex: 'obsCovPrior'
        },
        {
            text: 'stateCovPrior',
            flex:1,
            sortable: true,
            dataIndex: 'stateCovPrior'
        },
        {
            text: 'offTransPrior',
            flex:1,
            sortable: true,
            dataIndex: 'offTransPrior'
        },
        {
            text: 'onTransPrior',
            flex:1,
            sortable: true,
            dataIndex: 'onTransPrior'
        }
    ];

var priorGridColumns = gridColumns.concat([
        {
            text: 'edgeMarginalLogLik',
            flex:1,
            sortable: true,
            dataIndex: 'edgeMarginalLogLik'
        },
        {
            text: 'edgeTransLogLik',
            flex:1,
            sortable: true,
            dataIndex: 'edgeTransLogLik'
        },
        {
            text: 'measurementLogLik',
            flex:1,
            sortable: true,
            dataIndex: 'measurementLogLik'
        }
    ]);

  
Ext.onReady(function() {
  
  vehicleId = jQuery('#vehicleId').html();
  
  if (vehicleId) {
    $.ajax({
      url : '/api/traces',
      async : false,
      dataType : 'json',
      data : {
        vehicleId : vehicleId
      },
      success : function(data) {
        lines = data;
      }
    });
  }

  
  var postGrid = Ext.create('Ext.grid.Panel', {
    store: postParticleStore,
    autoShow: true,
    viewConfig: { forceFit: true },
    columns: gridColumns
  });
  
  var priorGrid = Ext.create('Ext.grid.Panel', {
    store: priorParticleStore,
    autoShow: true,
    viewConfig: { forceFit: true },
    columns: priorGridColumns
  });
  
  var sm = Ext.create('Ext.selection.CheckboxModel', {
    listeners: {
      select: function(rowModel, record, index, eOpts) {
        var layer = record.data.layer;
        
        if (!layer) {
          var path = record.data.data;
          var pathName = record.data.id;
          layer = renderPath(path.pathEdges, path.totalDistance,
              EdgeType.EVALUATED, true);
          record.data.layer = layer;
        }
        
        map.addLayer(layer);
      },
      deselect: function(rowModel, record, index, eOpts) {
        var layer = record.data.layer;
        map.removeLayer(layer);
      }
    }
  });
  
  var pathsGrid = Ext.create('Ext.grid.Panel', {
    store: pathsStore,
    selModel: sm,
    autoShow: true,
    viewConfig: { forceFit: true },
    columns: [{text: 'id', dataIndex:'id', flex: 1}, 
              {text: 'length', dataIndex: 'length', flex: 1},
              {text: 'edges', dataIndex: 'edges', flex: 1}
              ] 
  });
  
  var windowPanel = Ext.create('Ext.panel.Panel', {
    title: 'Particles',
    width: '100%',
    height: '100%',
    layout: 'border',
//    resizable: true,
    dockedItems: [{
      region: 'north',
      xtype: 'panel',
      layout: 'fit',
      items: [
              {
                fieldLabel: '',
                hideEmptyLabel: false,
                labelSeparator: '',
                labelWidth: 220,
                id: "recordSlider",
                xtype: 'sliderfield',
                minHeight: 20,
                value: 0,
                increment: 1,
                minValue: 0,
                maxValue: lines.length,          
                //        width: '50%',
                listeners: {
                  changecomplete: function(slider, thumb, oldValue, newValue) {
                    renderMarker(thumb);
                  }
                } 
             }
       ]
    }],
    items: [
      {
          title: 'Map',
          region: 'west',
          xtype: 'panel',
          split: true,
          width: '50%',
          collapsible: true,
          html: '<div id="map"></div>',
          id: 'mapContainer',
          layout: 'fit'
      },
      {
        region: 'center',
        width: '50%',
        minHeight: 400,
        xtype: 'tabpanel', 
        activeTab: 1,      
        items: [
          {
            title: 'Prior',
            layout: 'fit',
            html: '<div id="priorVisualControls"><span id="priorParticleMeans" class="jqlink">' 
              + 'show means</span> <span id="priorParticleParentMeans" class="jqlink" style="visibility:hidden;">'
              + 'show means</span> <span id="priorParticleEdges" class="jqlink">show edges</span><br></div>',
            items: [ priorGrid ]  
          },
          {
            title: 'Posterior',
            layout: 'fit',
            html: '<div id="postVisualControls"><span id="postParticleMeans" class="jqlink">' 
              + 'show means</span> <span id="postParticleParentMeans" class="jqlink" style="visibility:hidden;">'
              + 'show means</span> <span id="postParticleEdges" class="jqlink">show edges</span><br></div>',
            items: [ postGrid ]  
          },
          {
            title: 'Actual',
            layout: 'fit',
            id: 'actualResultsTab',
            html: '<div id="actualResults"></div>',
//            hidden: true
          }
        ]
      },
      {
        region: 'south',
        title: 'Paths',
        maxHeight: 400,
//        id: 'paths',
        layout: 'fit',
        items: [ pathsGrid ],
        split: true,
        collapsible: true,
      }
    ],
    renderTo: Ext.getBody()
  });
  
  Ext.EventManager.onWindowResize(function () {
    windowPanel.doComponentLayout();
  });

  map = new L.Map(jQuery('#map')[0]);

  var cloudmade = new L.TileLayer(cloudmadeUrl, cloudmadeOptions);
  map.addLayer(cloudmade);
  map.addLayer(pointsGroup);
  map.addLayer(edgeGroup);
  map.addLayer(inferredGroup);
  map.addLayer(actualGroup);
  map.addLayer(evaluatedGroup);
  map.addLayer(addedGroup);
  map.addLayer(allInfEdgesGroup);
  map.addLayer(allInfMeansGroup);

  map.setView(startLatLng, 17, true);

  var layersControl = new L.Control.Layers();
  layersControl.addBaseLayer(cloudmade, "base");
  layersControl.addOverlay(pointsGroup, "points");
  layersControl.addOverlay(edgeGroup, "edges");
  layersControl.addOverlay(inferredGroup, "inferred");
  layersControl.addOverlay(actualGroup, "actual");
  layersControl.addOverlay(evaluatedGroup, "evaluated");
  layersControl.addOverlay(addedGroup, "user added");
  layersControl.addOverlay(allInfEdgesGroup, "all inf. edges");
  layersControl.addOverlay(allInfMeansGroup, "all inf. means");

  map.addControl(layersControl);

  L.control.scale().addTo(map);

  $("#pause").hide();


  showParticlesMeans();
  showParticlesEdges();
  
  // TODO reimplement
  $("#addCoordinates").click(addCoordinates);
  $("#addEdge").click(addEdge);

});

function showParticlesEdges() {
  $("#postParticleEdges").hover(function() {
    var markers = $(this).data("particleEdges");
    if (markers) {
      allInfEdgesGroup.addLayer(markers);
      map.invalidateSize();
    }
  }, function() {
    var markers = $(this).data("particleEdges");
    if (markers) {
      allInfEdgesGroup.removeLayer(markers);
      map.invalidateSize();
    }
  });
}

function showParticlesMeans() {
  $("#postParticleMeans").hover(function() {
    var markers = $(this).data("particleMeans");
    if (markers) {
      allInfMeansGroup.addLayer(markers);
      map.invalidateSize();
    }
  }, function() {
    var markers = $(this).data("particleMeans");
    if (markers) {
      allInfMeansGroup.removeLayer(markers);
      map.invalidateSize();
    }
  });
  $("#priorParticleMeans").hover(function() {
    var markers = $(this).data("particleMeans");
    if (markers) {
      allInfMeansGroup.addLayer(markers);
      map.invalidateSize();
    }
  }, function() {
    var markers = $(this).data("particleMeans");
    if (markers) {
      allInfMeansGroup.removeLayer(markers);
      map.invalidateSize();
    }
  });
}

function getEdgeFromId(id) {
  var edge = null;
  $.ajax({
    url : '/api/segment?segmentId=' + id,
    dataType : 'json',
    async : false,
    cache : true,
    success : function(data) {
      edge = data;
    }
  });
  return edge;
}

function addEdge() {
  var id = jQuery('#edge_id').val();
  var edges = id.split(',');
  $.each(edges, function(index, value) {
    var edge = getEdgeFromId(value);
    drawEdge(edge, EdgeType.ADDED);
  });
  map.invalidateSize();
}

function drawCoords(lat, lon, popupMessage, pan, justMarker, color, opacity) {
  var latlng = new L.LatLng(parseFloat(lat), parseFloat(lon));

  if (!color)
    color = '#0c0';
  if (!opacity)
    opacity = 1;

  marker = new L.Circle(latlng, 10, {
    color : color,
    opacity : opacity,
    lat : parseFloat(lat),
    lon : parseFloat(lon),
    weight : 1
  });
  if (popupMessage != null)
    marker.bindPopup(popupMessage);

  if (justMarker)
    return marker;

  pointsGroup.addLayer(marker);

  if (pan)
    map.panTo(latlng);

  return marker;
}

//function drawProjectedCoords(x, y, popupMessage, pan, epsgCode) {
//  var latLon = convertToLatLon(new Proj4js.Point(x, y), epsgCode);
//  var marker = drawCoords(latLon.lat, latLon.lng, popupMessage, pan);
//  map.invalidateSize();
//
//  return marker;
//}

function addCoordinates() {

  var coordString = jQuery('#coordinate_data').val();
  var coordSplit = coordString.split(",");

  if (coordSplit.length == 2) {
    $.ajax({
      url : "/api/convertToLatLon",
      async : false,
      dataType : 'json',
      data : {
        x: coordSplit[0], 
        y: coordSplit[1]
      },
      success : function(data) {
        lat = data['x'];
        lon = data['y'];
  
        var latlng = new L.LatLng(parseFloat(lat), parseFloat(lon));
        var new_marker = new L.Circle(latlng, 10, {
          color : '#0c0',
          lat : parseFloat(lat),
          lon : parseFloat(lon)
        });
        pointsGroup.addLayer(new_marker);
        addedGroup.addLayer(new_marker);
        map.panTo(latlng);
        map.invalidateSize();
      }
    });
  }

}

/*
 * -No longer relevant
 * Xian 1980 / Gauss-Kruger zone 21 EPSG:2335 isn't saved locally. if we omit
 * this, then there will be an error followed by a query that should obtain this
 * information. TODO what about different zones? how will they be
 * detected/handled?
 */
//Proj4js.defs["EPSG:2335"] = "+proj=tmerc +lat_0=0 +lon_0=123 +k=1 +x_0=21500000 +y_0=0 +a=6378140 +b=6356755.288157528 +units=m +no_defs";
Proj4js.defs["EPSG:32618"] = "+proj=utm +zone=18 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";

var dest = new Proj4js.Proj("EPSG:4326");
//var source = new Proj4js.Proj("EPSG:2335");

function truncate(_value) {
  if (_value < 0)
    return Math.ceil(_value);
  else
    return Math.floor(_value);
}

function getUTMzone(lon) {
  if (lon < -180 || lon > 180)
    return null;

  var lonZone = truncate((lon + 180) / 6);

  if (lonZone == 60)
    lonZone--;
  return lonZone + 1;
}

function convertToLatLon(srcPoint, epsgCode) {
  var point = new Proj4js.Point(srcPoint.x, srcPoint.y);
  var source = new Proj4js.Proj(epsgCode);
  Proj4js.transform(source, dest, point);
  return new L.LatLng(point.y, point.x);
}

function drawResults(mean, major, minor, pointType, epsgCode) {

  var color;
  var fill;
  var groupType;
  if (pointType == PointType.INFERRED_FREE) {
    color = 'red';
    fill = false;
    groupType = inferredGroup;
  } else if (pointType == PointType.INFERRED_EDGE) {
    color = 'red';
    fill = true;
    groupType = inferredGroup;
  } else if (pointType == PointType.ACTUAL_FREE) {
    color = 'black';
    fill = false;
    groupType = actualGroup;
  } else if (pointType == PointType.ACTUAL_EDGE) {
    color = 'black';
    fill = true;
    groupType = actualGroup;
  }

  var meanCoords = convertToLatLon(mean, epsgCode);

  if (major && minor) {
    var majorLatLon = convertToLatLon(major, epsgCode);
    var majorAxis = new L.Polyline([ meanCoords, majorLatLon ], {
      fill : true,
      color : '#c00'
    });

    pointsGroup.addLayer(majorAxis);
    groupType.addLayer(majorAxis);

    var minorLatLon = convertToLatLon(minor, epsgCode);
    var minorAxis = new L.Polyline([ meanCoords, minorLatLon ], {
      fill : true,
      color : '#c0c'
    });

    pointsGroup.addLayer(minorAxis);
    groupType.addLayer(minorAxis);
  }

  var mean = new L.Circle(meanCoords, 5, {
    fill : fill,
    color : color,
    opacity : 1.0
  });

  pointsGroup.addLayer(mean);
  groupType.addLayer(mean);

}

PointType = {
  INFERRED_FREE : 0,
  INFERRED_EDGE : 1,
  ACTUAL_FREE : 2,
  ACTUAL_EDGE : 3
};

function renderMarker(recordNumber) {
  if (lines && recordNumber >= 0 && recordNumber < lines.length) {
    
    var recordData = lines[recordNumber];
    
    pointsGroup.clearLayers();
    edgeGroup.clearLayers();
    actualGroup.clearLayers();
    inferredGroup.clearLayers();
    addedGroup.clearLayers();
    evaluatedGroup.clearLayers();
    allInfEdgesGroup.clearLayers();
    allInfMeansGroup.clearLayers();

    /*
     * Draw lines first
     */
    renderGraph(recordNumber);

    /*
     * Show particle cloud and paths
     */
    if (recordData.infResults) {
      var edgeMarkers = $("#postParticleEdges").data("particleEdges");
      if (edgeMarkers) {
        allInfEdgesGroup.addLayer(edgeMarkers);
        map.invalidateSize();
      }
      var markers = $("#postParticleMeans").data("particleMeans");
      if (markers) {
        allInfMeansGroup.addLayer(markers);
        map.invalidateSize();
      }
    }

    if (recordData.actualResults) {
      var pointType = PointType.ACTUAL_FREE;

      if (recordData.actualResults.pathSegments.length > 0) {
        pointType = PointType.ACTUAL_EDGE;
      }

      var results = recordData.actualResults;
      drawResults(results.meanCoords, results.majorAxisCoords,
          results.minorAxisCoords, pointType, results.espgCode);
    }

    var obsCoords = convertToLatLon(recordData.observedPoint,
        recordData.observedPoint.epsgCode);
    //    var obsCoords = new L.LatLng(parseFloat(lines[i].observedCoords.x),
    //        parseFloat(lines[i].observedCoords.y));
    var obs = new L.Circle(obsCoords, 10, {
      fill : true,
      color : 'grey',
      opacity : 1.0
    });
    pointsGroup.addLayer(obs);

    map.panTo(obsCoords);

    var theDate = new Date(parseFloat(recordData.time));
    Ext.getCmp('recordSlider').setFieldLabel(
        theDate.toUTCString() + ' - ' + recordData.time + ' (' + recordNumber + ')');
    return false;
  } else {
    clearInterval(interval);
    return true;
  }
}

EdgeType = {
  ACTUAL : 0,
  INFERRED : 1,
  EVALUATED : 2,
  ADDED : 3,
  INFERRED_ALL : 4
};

function drawEdge(edge, edgeType, layerOnly) {

  var result = null;
  var data = edge;
  var velocity = edge.velocity;
  var avg_velocity = Math.abs(velocity);

  var color;
  var weight = 5;
  var opacity = 0.7;

  var groupType;
  if (edgeType == EdgeType.INFERRED) {
    if (avg_velocity != null && avg_velocity < MAX_SPEED)
      color = '#' + getColor(avg_velocity / MAX_SPEED);
    else
      color = "red";

    weight = 10;
    opacity = 0.3;
    groupType = inferredGroup;
  } else if (edgeType == EdgeType.ACTUAL) {
    color = "black";
    weight = 2;
    opacity = 1.0;
    groupType = actualGroup;
  } else if (edgeType == EdgeType.EVALUATED) {
    color = "blue";
    weight = 20;
    opacity = 0.2;
    groupType = evaluatedGroup;
  } else {
    color = "green";
    weight = 20;
    opacity = 0.2;
    groupType = addedGroup;
  }

  var geojson = new L.GeoJSON();
  geojson.on('featureparse', function(e) {
    e.layer.setStyle({
      color : e.properties.color,
      weight : weight,
      opacity : opacity
    });
    if (e.properties && e.properties.popupContent) {
      e.layer.bindPopup(e.properties.popupContent);
    }
  });

  var escName = data.name.replace(/([\\<\\>'])/g, "");

  data.geom.properties = {
    popupContent : escName,
    color : color
  };

  var layers = new Array(geojson);
  
  data.geom.coordinates.forEach(function(element, index, array) {  
    array[index] = [element[1], element[0]] })
  
  geojson.addData(data.geom);

  var arrowhead;
  var angle = data.angle;
  if (angle != null) {
    var lonlat = data.geom.coordinates[data.geom.coordinates.length - 1];
    var myIcon = new L.icon({
      iconUrl : '/public/images/tab_right.png',
      shadowUrl : null,
      shadowSize : null,
      iconSize : [ 80, 30 ]
    });
    arrowhead = new L.Marker.Compass(new L.LatLng(lonlat[1], lonlat[0]), {
      icon : myIcon,
      clickable : false
    });
    layers.push(arrowhead);
  }

  result = new L.LayerGroup(layers);
  if (!layerOnly) {
    // edgeGroup.addLayer(result);
    groupType.addLayer(result);
  }

  if (arrowhead)
    arrowhead.setIconAngle(-angle);

  map.invalidateSize();

  return result;
}

MyIcon = L.Icon.extend({
  iconUrl : '/public/images/tab_right.png',
  shadowUrl : null,
  shadowSize : null,
  iconAnchor : new L.Point(10, 22)
});

function createMatrixString(matrix, isVector) {
  var resStr = "";
  var covLen = matrix.length;
  var cols = isVector ? covLen : Math.sqrt(covLen);
  $.each(matrix, function(index, data) {
    //var tmpStr = parseFloat(data).toFixed(2);
    var tmpStr = parseFloat(data).toPrecision(5);
    if (index == 0) {
      //tmpStr = (isVector ? "[" : "[[") + tmpStr;
      tmpStr = "[" + tmpStr;
      if (index + 1 == covLen) {
        resStr = tmpStr + "]";
      } else {
        resStr = tmpStr + ",";
      }
    } else if (index + 1 == covLen) {
      //tmpStr = tmpStr + (isVector ? "]" : "]]");
      tmpStr = tmpStr + "]";
      resStr = resStr + tmpStr;
    } else if ((index + 1) % cols == 0) {
      //tmpStr = tmpStr + "],<br>[";
      tmpStr = tmpStr + ",<br>";
      resStr = resStr + tmpStr;
    } else {
      resStr = resStr + tmpStr + ",";
    }
  });

  return resStr;
}

function getPathName(pathSegmentIds) {
  // var resStr = new Array();
  // $.each(pathSegmentIds, function(index, data) {
  // resStr.push(data[0]);
  // });

  return paths[arrayHash(pathSegmentIds)];
}

function renderParticles(recordNumber, isPrior) {
  $.ajax({
    url : '/api/traceParticleRecord',
    async : false,
    dataType : 'json',
    data : {
      vehicleId : vehicleId,
      recordNumber : recordNumber,
      particleNumber : -1,
      withParent : true,
      isPrior : isPrior
    },
    success : function(data) {

      var particleEdgesDiv = null;
      var particleMeansDiv = null;
      var particleParentMeansDiv = null;
      if (!isPrior) {
        postParticleStore.clearData();
        particleMeansDiv = jQuery("#postParticleMeans");
        particleEdgesDiv = jQuery("#postParticleEdges");
        particleParentMeansDiv = jQuery("#postParticleParentMeans");
      } else {
        priorParticleStore.clearData();
        particleMeansDiv = jQuery("#priorParticleMeans");
        particleEdgesDiv = jQuery("#priorParticleEdges");
        particleParentMeansDiv = jQuery("#priorParticleParentMeans");
      }

      var particleMeans = new L.LayerGroup();
      particleMeansDiv.data("particleMeans", particleMeans);

      var particleEdges = new L.LayerGroup();
      particleEdgesDiv.data("particleEdges", particleEdges);

      var particleParentMeans = new L.LayerGroup();
      particleParentMeansDiv.data("particleParentMeans", particleParentMeans);

      if (data == null)
        return;

      jQuery.each(data,
          function(_, particleData) {

            var type = "post";
            if (isPrior) {
              type = "prior";
            }

            var epsgCode = particleData.particle.observedPoint.epsgCode;
            globalEpsgCode = epsgCode;

            var particleEntry = createParticleEntry('current_' + type,
                epsgCode, particleData.isBest, particleData.weight, particleData.particleNumber,
                particleData.particle, particleMeans, particleEdges);
            
            if (isPrior) {
              priorParticleStore.loadRawData([particleEntry], true);
            } else {
              postParticleStore.loadRawData([particleEntry], true);
            }

            if (particleData.parent) {
              // TODO add parents
//              var parentEntry = createParticleEntry('parent_' + type,
//                  epsgCode, null, null, null,
//                  particleData.parent, particleParentMeans, particleEdges);
            }
          });
    }
  });
}

/**
fields: [
         {name: 'number'},
         {name: 'weight'},
         {name: 'count'},
         {name: 'mean'},
         {name: 'edge'},
         {name: 'distMoved'},
         {name: 'path'},
         {name: 'state'},
         {name: 'obsCovPrior'},
         {name: 'stateCovPrior'},
         {name: 'offTransPrior'},
         {name: 'onTransPrior'}
         ] 
*/
function createParticleEntry(particleTypeId, epsgCode, particleIsBest, particleWeight, particleNumber, 
    particleData, particleMeans, particleEdges) {
  
  var resultArray = {};
  
  resultArray['number'] = particleNumber;
  resultArray['weight'] = particleWeight;
  resultArray['count'] = particleData.infResults.particleCount;
  
  var priorPredResults = particleData.infResults.priorPredictiveResults;
  if (priorPredResults) {
    resultArray['edgeMarginalLogLik'] = priorPredResults.edgeMarginalLogLik;
    resultArray['edgeTransLogLik'] = priorPredResults.edgeTransLogLik;
    resultArray['measurementLogLik'] = priorPredResults.measurementLogLik;
  }
  
  var locLinkName = particleTypeId + '_particle_' + particleNumber + '_mean';
  var locLink = createStateLink(locLinkName, particleData.infResults.pathState);
  var meanLatLon = convertToLatLon(particleData.infResults.meanCoords, epsgCode);
  
  particleMeans.addLayer(drawCoords(meanLatLon.lat,
      meanLatLon.lng, null, false, true, null, 0.5
          + particleWeight
          * particleData.infResults.particleCount));
  
  resultArray['mean'] = locLink.prop('outerHTML');

  var edgeDesc = "free";
  var edgeId = particleData.infResults.inferredEdge.id;
  if (edgeId != null && edgeId != -1) {
    edgeDesc = edgeId
        + " ("
        + parseFloat(particleData.infResults.inferredEdge.length)
            .toFixed(2) + ")";
  }
  
  
  var edgeLinkName = particleTypeId + '_particle_' + particleNumber + '_edge';
  var edgeLink = jQuery('<a name="' + edgeLinkName + '" title="' + edgeDesc
      + '" style="color : black" href="javascript:void(0)">' + edgeDesc
      + '</a>');
  var edge = particleData.infResults.inferredEdge;
  if (edge != null) {
    createHoverLineLink(edgeLink, edge);
  }
  
  resultArray['edge'] = edgeLink.prop('outerHTML');
  

  particleEdges.addLayer(renderPath(
      particleData.infResults.traveledSegments,
      particleData.infResults.pathDirection, EdgeType.INFERRED_ALL,
      true));

  var stateSample = particleData.infResults.currentStateSample;
  if (stateSample !== null) {
    var smplLocLinkName = particleTypeId + '_stateSample_' + particleNumber
        + '_mean';
    var stateLink = createStateLink(smplLocLinkName, stateSample);
    resultArray['currentStateSample'] = stateLink.prop('outerHTML');
  }
  
  var prevStateSample = particleData.infResults.prevStateSample;
  if (prevStateSample !== null) {
    var smplLocLinkName = particleTypeId + '_stateSample_' + particleNumber
        + '_mean';
    var stateLink = createStateLink(smplLocLinkName, stateSample);
    resultArray['prevStateSample'] = stateLink.prop('outerHTML');
  }

  var pathName = getPathName(getEdgeIdsFromSegments(particleData.infResults.pathSegments));
  var pathData = $('#' + pathName).data('path');
  if (pathData) {
    var pathLikelihood = parseFloat(pathData.totalLogLikelihood).toFixed(2);
    resultArray['pathInfo'] = pathName + ', ' + pathLikelihood;
  }
  resultArray['distMoved'] =
      parseFloat(particleData.infResults.distanceFromPreviousState).toFixed(2);
  
  resultArray['path'] = getEdgeIdsFromSegments(particleData.infResults.traveledSegments).toString();

  var obsCov;
  if (particleData.infResults.obsCovarPrior) {
    var obsCovarPrior = particleData.infResults.obsCovarPrior;
    var obsCovPriorMean = $V(obsCovarPrior.scale).multiply(
        1 / (obsCovarPrior.dof - 2 - 1)).elements;
    obsCov = createMatrixString(obsCovPriorMean, false);
  } else {
    obsCov = createMatrixString(particleData.infResults.obsCovariance,
        false);
  }
  resultArray['obsCovPrior'] = obsCov;

  var stateCov;
  if (particleData.infResults.onRoadCovarPrior) {
    var stateCovarPrior;
    if (Math.sqrt(particleData.infResults.pathState.covariance.length) > 2) {
      stateCovarPrior = particleData.infResults.offRoadCovarPrior;
    } else {
      stateCovarPrior = particleData.infResults.onRoadCovarPrior;
    }
    var stateCovPriorMean = $V(stateCovarPrior.scale)
        .multiply(
            1 / (stateCovarPrior.dof - Math.sqrt(stateCovarPrior.scale.length) - 1)).elements;
    stateCov = createMatrixString(stateCovPriorMean, false);
  } else {
    stateCov = createMatrixString(
        particleData.infResults.pathState.covariance, false);
  }
  resultArray['stateCovPrior'] = stateCov;

  var offTransPrior = createMatrixString(
      particleData.infResults.offRoadTransProbsPriorParams, true);
  var offTransProbs = createMatrixString(
      particleData.infResults.offRoadTransProbs, true);
  offTransPrior = offTransPrior + " (" + offTransProbs + ")"
  
  resultArray['offTransPrior'] = offTransPrior;

  var onTransPrior = createMatrixString(
      particleData.infResults.onRoadTransProbsPriorParams, true);
  var onTransProbs = createMatrixString(
      particleData.infResults.onRoadTransProbs, true);
  onTransPrior = onTransPrior + " (" + onTransProbs + ")"
  
  resultArray['onTransPrior'] = onTransPrior;

  return resultArray;
}

function createStateLink(id, stateObj) {
  var coordPair = stateObj.stateLoc[0] + ',' + stateObj.stateLoc[1];
  var stateVec = createMatrixString(stateObj.state, true);
  var stateLink = jQuery('<a name="' + id + '" title="'
      + coordPair
      + '" style="color : black" href="javascript:void(0)">' + stateVec + '</a>');
  
  var meanLoc = {
      x: stateObj.stateLoc[0],
      y:  stateObj.stateLoc[1]
  };
  var meanLatLon = convertToLatLon(meanLoc, globalEpsgCode);
  createHoverPointLink(stateLink, meanLatLon);
  
  return stateLink;
}


function createHoverLineLink(link, edge) {
  if (edge.id < 0)
    return;
  var selector = 'a[name=' + link.prop('name') + ']';
  
  $(document).off("mouseenter mouseleave", selector);
  
  $(document).on({
    mouseenter: function(event) {
      var localEdge = event.data;
      var edge = this.edge;
      if (edge == null) {
        edge = drawEdge(localEdge, EdgeType.INFERRED);
        this.edge = edge;
      } else {
        edgeGroup.addLayer(edge);
        addedGroup.addLayer(edge);
        map.invalidateSize();
      }
    }, 
    mouseleave: function(event) {
      var edge = this.edge;
      if (edge != null) {
        edgeGroup.removeLayer(edge);
        addedGroup.removeLayer(edge);
        map.invalidateSize();
      }
    }
  }, selector, edge);
}

function createHoverPointLink(link, latlon) {
  var selector = 'a[name=' + link.prop('name') + ']';
  $(document).off("mouseenter mouseleave", selector);
  $(document).on({
    mouseenter: function(event) {
        var localLoc = event.data;
        var marker = this.marker;
        if (marker == null) {
          marker = drawCoords(localLoc.lat, localLoc.lng, null, false, false,
              'red');
          this.marker = marker;
        } else {
          pointsGroup.addLayer(marker);
          addedGroup.addLayer(marker);
          map.invalidateSize();
        }
      }, 
    mouseleave: function(event) {
      var marker = this.marker;
      if (marker != null) {
        pointsGroup.removeLayer(marker);
        addedGroup.removeLayer(marker);
        map.invalidateSize();
      }
    }
  }, selector, latlon);
}

function arrayHash(array) {
  var prime = 31;
  var result = 1;
  $.each(array, function(index, value) {
    result = prime * result + parseInt(value);
  });
  return result;
}

function getEdgeIdsFromSegments(segments) {
  var edgeIds = new Array();
  $.each(segments, function(index, data) {
    edgeIds.push(data.id);
  });
  return edgeIds;
}

function renderGraph(recordNumber) {
  paths = {};
  var recordData = lines[recordNumber];
  if (recordData.infResults) {
    var pathList = jQuery("#paths");
    pathList.empty();

    var emptyOption = jQuery('<option id="none">none</option>');
    var startPointsOption = jQuery('<option id="startEdges">startEdges</option>');
    var endPointsOption = jQuery('<option id="endEdges">endEdges</option>');
    pathList.append(emptyOption);
    pathList.append(startPointsOption);
    pathList.append(endPointsOption);

    // TODO FIXME must make a separate call to get this
    // info. no longer contained in every particle.

    var evaluatedPaths = null;
    $
        .ajax({
          url : "/api/evaluatedPaths?vehicleId=" + vehicleId + "&recordNumber="
              + recordNumber,
          dataType : 'json',
          async : false,
          success : function(data) {
            evaluatedPaths = data;
          }
        });

    if (evaluatedPaths != null && evaluatedPaths.length > 0) {
      var pathDataArray = new Array();
      
      for ( var k in evaluatedPaths) {

        var pathName = 'path' + k;
        var pathData = evaluatedPaths[k];
        var edgeIds = getEdgeIdsFromSegments(evaluatedPaths[k].pathEdges);
        paths[arrayHash(edgeIds)] = pathName;
        
        var pathObject = {
            id: pathName,
            length: parseFloat(evaluatedPaths[k].totalDistance).toFixed(2),
            edges: edgeIds.toString(),
            data: pathData
        };
        
        pathDataArray.concat(pathObject);

      }
      
      pathsStore.loadRawData(pathDataArray, false);
    }
  }

  renderParticles(recordNumber, false);
  renderParticles(recordNumber, true);

  if (recordData.actualResults) {
    renderPath(recordData.actualResults.traveledSegments,
        recordData.actualResults.pathDirection, EdgeType.ACTUAL);

//    var actualResultsTab = Ext.getCmp('actualResultsTab').tab;
//    actualResultsTab.show();

    var actualResultsDiv = jQuery("#actualResults");
    actualResultsDiv.empty();

    var actualResultsList = jQuery("<ul></ul>");
    var state = createMatrixString(recordData.actualResults.pathState.state, true);
    actualResultsList.append('state:<ul><li>' + state + '</li></ul>');
    //    var stateCov = createMatrixString(lines[i].actualResults.stateCovariance, false);
    //    actualResultsList.append('<li>stateCov:' + stateCov + '</li>');
    var obsCov = createMatrixString(recordData.actualResults.obsCovariance, false);
    actualResultsList.append('obsCov:<ul><li>' + obsCov + '</li></ul>');
    var onRoadCov = createMatrixString(
        recordData.actualResults.onRoadStateCovariance, false);
    actualResultsList.append('onRoadCov:<ul><li>' + onRoadCov + '</li></ul>');
    var offRoadCov = createMatrixString(
        recordData.actualResults.offRoadStateCovariance, false);
    actualResultsList.append('offRoadCov:<ul><li>' + offRoadCov + '</li></ul>');
    var traveledSegments = getEdgeIdsFromSegments(recordData.actualResults.traveledSegments);
    actualResultsList.append('traveledSegments:<ul><li>' + traveledSegments
        + '</li></ul>');
    actualResultsDiv.append(actualResultsList);
  }
}

function renderPath(pathSegments, pathDirection, edgeType, layerOnly) {
  var color;
  var weight = 5;
  var opacity = 0.7;

  var groupType;
  if (edgeType == EdgeType.INFERRED) {
    color = "red";
    weight = 10;
    opacity = 0.3;
    groupType = inferredGroup;
  } else if (edgeType == EdgeType.ACTUAL) {
    color = "black";
    weight = 2;
    opacity = 1.0;
    groupType = actualGroup;
  } else if (edgeType == EdgeType.EVALUATED) {
    color = "blue";
    weight = 20;
    opacity = 0.3;
    groupType = evaluatedGroup;
  } else if (edgeType == EdgeType.ADDED) {
    color = "green";
    weight = 20;
    opacity = 0.1;
    groupType = addedGroup;
  } else if (edgeType == EdgeType.INFERRED_ALL) {
    color = "green";
    weight = 10;
    opacity = 0.10;
    groupType = allInfEdgesGroup;
  }

  var segments = $.extend(true, [], pathSegments);
  if (pathDirection < 0) {
    segments.reverse();
  }

  var latLngs = new Array();
  var justIds = new Array();
  for ( var j in segments) {
    var segmentInfo = segments[j];
    var edgeId = segmentInfo.id;

    if (edgeId > -1) {
      if (j > 0 && j % 5 == 0) {
        justIds.push('<br>' + edgeId);
      } else {
        justIds.push(edgeId);
      }

      for ( var k in segmentInfo.geom.coordinates) {
        latLngs.push(new L.LatLng(segmentInfo.geom.coordinates[k][0],
            segmentInfo.geom.coordinates[k][1]));
      }
    }
  }

  var polyline = new L.Polyline(latLngs, {
    color : color,
    weight : weight,
    opacity : opacity
  });

  polyline.bindPopup(justIds.toString());

  if (!layerOnly) {
    // edgeGroup.addLayer(polyline);
    groupType.addLayer(polyline);
    map.invalidateSize();
  }

  return polyline;
}

L.Marker.Compass = L.Marker
    .extend({
      _reset : function() {

        var pos = this._map.latLngToLayerPoint(this._latlng).round();

        L.DomUtil.setPosition(this._icon, pos);
        if (this._shadow) {
          L.DomUtil.setPosition(this._shadow, pos);
        }

        if (this.options.iconAngle) {
          this._icon.style.WebkitTransform = this._icon.style.WebkitTransform
              + ' rotate(' + this.options.iconAngle + 'deg)';
          this._icon.style.MozTransform = 'rotate(' + this.options.iconAngle
              + 'deg)';
          this._icon.style.MsTransform = 'rotate(' + this.options.iconAngle
              + 'deg)';
          this._icon.style.OTransform = 'rotate(' + this.options.iconAngle
              + 'deg)';
        }

        this._icon.style.zIndex = pos.y;
      },

      _initIcon : function() {
        L.Marker.prototype._initIcon.call(this)
        this._reset();
      },

      onAdd : function(e) {
        L.Marker.prototype.onAdd.call(this, e);
        if (this._map)
          this._reset();
      },

      setIconAngle : function(iconAngle) {

        if (this._map) {
          this._removeIcon();
        }

        this.options.iconAngle = iconAngle;

        if (this._map) {
          this._initIcon();
          this._reset();
        }
      }

    });

function getColor(f) {
  var n = Math.round(100 * f);

  var red = (255 * n) / 100;
  var green = (255 * (100 - n)) / 100;
  var blue = 0;

  var rgb = blue | (green << 8) | (red << 16);

  return rgb.toString(16);
}

/*
 * parse Well Known Text (WKT) and return a PolyLine, MultiPolyline, et cetera
 * params:
 * - wkt, String WKT to be parsed
 * - options, object to be passed to the new feature as it is constructed, e.g. fillColor, strokeWidth
 * returns:
 * - an instance of a L.Path subclass, e.g. L.Polyline
 *
 * The supported options depends on the type of feature found   http://leaflet.cloudmade.com/reference.html#path
 */
L.WKTtoFeature = function(wkt, options) {
  // really, this is a wrapper to the WKTtoFeature.parse* functions
  wkt = wkt.replace(/^\s*/g, '').replace(/\s*$/, '');
  if (wkt.indexOf('LINESTRING') == 0)
    return L.WKTtoFeature.parseLinestring(wkt, options);
  if (wkt.indexOf('MULTILINESTRING') == 0)
    return L.WKTtoFeature.parseMultiLinestring(wkt, options);
};

L.WKTtoFeature.parseLinestring = function(wkt, options) {
  // split on , to get vertices. handle possible spaces after commas
  var verts = wkt.replace(/^LINESTRING\s*\(/, '').replace(/\)$/, '').split(
      /,\s*/);

  // collect vertices into a line
  var line = [];
  for ( var vi = 0, vl = verts.length; vi < vl; vi++) {
    var lng = parseFloat(verts[vi].split(" ")[1]);
    var lat = parseFloat(verts[vi].split(" ")[0]);
    line[line.length] = convertToLatLon({
      x : lng,
      y : lat
    }, globalEpsgCode);
    //        line[line.length] = new L.LatLng(lat,lng);
  }

  // all set, return the Polyline with the user-supplied options/style
  var feature = new L.Polyline(line, options);
  return feature;
}

L.WKTtoFeature.parseMultiLinestring = function(wkt, options) {
  // some text fixes
  wkt = wkt.replace(/^MULTILINESTRING\s*\(\(/, '').replace(/\)\)$/, '');

  // split by () content to get linestrings, split linestrings by commas to get vertices
  var multiline = [];
  var getLineStrings = /\((.+?)\)/g;
  var getVerts = /,\s*/g;
  while (lsmatch = getLineStrings.exec(wkt)) {
    var line = [];
    var verts = lsmatch[1].split(getVerts);
    for ( var i = 0; i < verts.length; i++) {
      var lng = parseFloat(verts[i].split(" ")[1]);
      var lat = parseFloat(verts[i].split(" ")[0]);
      line[line.length] = convertToLatLon({
        x : lng,
        y : lat
      }, globalEpsgCode);
      //            line[line.length] = new L.LatLng(lat,lng);
    }
    multiline[multiline.length] = line;
  }

  // all set, return the MultiPolyline with the user-supplied options/style
  var feature = new L.MultiPolyline(multiline, options);
  return feature;
}

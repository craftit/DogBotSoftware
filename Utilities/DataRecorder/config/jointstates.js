/* global _ */

/*
 * Complex scripted dashboard
 * This script generates a dashboard object that Grafana can load. It also takes a number of user
 * supplied URL parameters (in the ARGS variable)
 *
 * Return a dashboard object, or a function
 *
 * For async scripts, return a function, this function must take a single callback function as argument,
 * call this callback function with the dashboard object (look at scripted_async.js for an example)
 */

'use strict';

// accessible variables in this scope
var window, document, ARGS, $, jQuery, moment, kbn;

// Setup some variables
var dashboard;

// All url parameters are available via the ARGS object
var ARGS;
var debugme = true;

// Initialize a skeleton with nothing but a rows array and service object
dashboard = {
  rows : [],
};

// Set a title
dashboard.title = 'DogBot Joint States';

// Set default time
// time can be overridden in the url using from/to parameters, but this is
// handled automatically in grafana core during dashboard initialization
dashboard.time = {
  from: "now-6h",
  to: "now"
};

/*dashboard.__inputs = [
    {
      "name": "DS_DOGBOT",
      "label": "dogbot",
      "description": "",
      "type": "datasource",
      "pluginId": "postgres",
      "pluginName": "PostgreSQL"
    }
  ]*/


var jointnames = [];
var jointids = [];
var lr = ["l","r"];
var leftright = ["Left","Right"];
var fb = ["f","b"];
var frontback = ["Front","Back"];
var jointtypes = ["Knee","Pitch","Roll"];

for (var i=0; i<2; i++){
  for (var j=0; j<2; j++){
    for (var k=0; k<3; k++){
      jointnames.push(frontback[i] + " " + leftright[j] + " " + jointtypes[k])
      jointids.push(fb[i] + lr[j] + jointtypes[k].toLowerCase() )
    }
  }
}

if(debugme){
  console.log("Joints: " + String(jointnames))
  console.log("Joint IDs: " + String(jointids))
}

var rows = jointnames.length;

//in case we want fewer via the url
if(!_.isUndefined(ARGS.rows)) {
  rows = parseInt(ARGS.rows, 12);
}

function JointPanel(jnt){
  var panel = {
              "aliasColors": {},
              "bars": false,
              "dashLength": 10,
              "dashes": false,
              "datasource": "dogbot",
              //"datasource": null,
              "fill": 0,
//               "gridPos": {
//                 "h": 13,
//                 "w": 23,
//                 "x": 0,
//                 "y": 0
//               },
              "id": jnt+1,
              "legend": {
                "avg": false,
                "current": false,
                "max": true,
                "min": true,
                "show": true,
                "total": false,
                "values": true
              },
              "lines": true,
              "linewidth": 2,
              "links": [],
              "nullPointMode": "null",
              "percentage": false,
              "pointradius": 5,
              "points": false,
              "renderer": "flot",
              "seriesOverrides": [
                {
                  "alias": "effort",
                  "yaxis": 2
                }
              ],
              "spaceLength": 10,
              "stack": false,
              "steppedLine": false,
              "targets": [
                {
                  "alias": "",
                  "format": "time_series",
                  "hide": false,
                  "rawSql": "SELECT\n  $__time(logtime),\n  value as \"command\"\nFROM\n  position_command\nWHERE\n  $__timeFilter(logtime) and sourceid='" + jointids[jnt] + "'\n",
                  "refId": "A"
                },
                {
                  "alias": "",
                  "format": "time_series",
                  "hide": false,
                  "rawSql": "SELECT\n  $__time(logtime),\n  position, velocity, effort\nFROM\n  joint_state\nWHERE\n  $__timeFilter(logtime) and sourceid='" + jointids[jnt] + "'\n",
                  "refId": "B"
                }
              ],
              "thresholds": [],
              "timeFrom": null,
              "timeShift": null,
              "title": jointnames[jnt],
              "tooltip": {
                "shared": true,
                "sort": 0,
                "value_type": "individual"
              },
              "type": "graph",
              "xaxis": {
                "buckets": null,
                "mode": "time",
                "name": null,
                "show": true,
                "values": []
              },
              "yaxes": [
                {
                  "format": "short",
                  "label": null,
                  "logBase": 1,
                  "max": null,
                  "min": null,
                  "show": true
                },
                {
                  "format": "short",
                  "label": null,
                  "logBase": 1,
                  "max": null,
                  "min": null,
                  "show": true
                }
              ]
            }
   return panel;
}

var panels = []

for (var i = 0; i < rows; i++) {
  var panel = JointPanel(i);
  panels.push( panel );
}

dashboard.rows.push({"panels": panels})

if (debugme){
  console.log(JSON.stringify(dashboard))
}

return dashboard;

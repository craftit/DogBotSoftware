# Dogbot Utilities

## Dogbot ROS-SQL Logger

To enable logging of Dogbot's data from ROS topics into a [PostgreSQL] database, you can add a call to a logger node into your launch files. For example, the Gazebo launch file contains:

```XML
  <include if="$(arg postgres_log)"  file="$(find dogbot_utilities)/config/logger.launch">
    <arg name="logging_config" value="sim_logging.yaml"/>
  </include>
```

`dogbot_logging.yaml` will record the ROS topics from a physical Dogbot
`sim_logging.yaml` will record ROS topics from a Gazebo simulation

When inspecting data via psql, don't forget to use the named database with e.g. `\c dogbot;`


[PostgreSQL]: https://www.postgresql.org/
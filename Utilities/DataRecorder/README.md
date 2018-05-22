# DogBot Data Recorder

A utilty to place DogBot data into a relational database, [PostgreSQL], for visualisation and analysis.

## Install and Build the Database

Install PostgreSQL, v9.5 or higher, create a suitable named database (the configuration files assume 'dogbot' as a default), and run:
`psql -U username dogbot < ./config/postgres.sql`

replacing `username` with suitable access credentials


## Set up Grafana

create a Postgres datasource with the name 'dogbot' - MORE...

## Access Scripted Dashboards

The scripted dashboards need copying to ```/usr/share/grafana/public/dashboards/```, after which they are available in the local grafana session via urls such as:
```http://127.0.0.1:3000/dashboard/script/jointstates.js```

MORE TO FOLLOW

[PostgreSQL]: https://www.postgresql.org/
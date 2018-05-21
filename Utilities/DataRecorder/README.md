# DogBot Data Recorder

A utilty to place DogBot data into a relational database, [PostgreSQL], for visualisation and analysis.

## Install and Build the Database

Install PostgreSQL, v9.5 or higher, create a suitable named database (the configuration files assume 'dogbot' as a default), and run:
`psql -U username dogbot < ./config/postgres.sql`

replacing `username` with suitable access credentials


[PostgreSQL]: https://www.postgresql.org/
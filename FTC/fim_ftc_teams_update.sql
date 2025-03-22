UPDATE
	fim_ftc_teams
SET
	team_number = trim(team_number),
	affiliation = trim(affiliation),
	city = trim(city),
	team_name = trim(team_name);

UPDATE fim_ftc_teams tableB
INNER JOIN us_cities tableA ON tableB.city = tableA.city
SET tableB.latitude = tableA.lat, tableB.longitude = tableA.lng
WHERE tableA.state_name = 'Michigan';

update fim_ftc_teams set latitude = '42.8471381', longitude = '-83.0825578' where city = 'Bruce Twp' and latitude is null;
update fim_ftc_teams set latitude = '42.5699142', longitude = '-83.5410233' where city = 'Commerce Township' and latitude is null;
update fim_ftc_teams set city = 'West Bloomfield',latitude = '42.5720585', longitude = '-83.4593845' where (city = 'W Bloomfield' or city = 'West Bloomfield') and latitude is null;
update fim_ftc_teams set latitude = '43.0415661', longitude = '-82.5184658' where city = 'Fort Gratiot' and latitude is null;
update fim_ftc_teams set latitude = '42.4957616', longitude = '-82.9330589' where city = 'Saint Clair Shores' and latitude is null;
update fim_ftc_teams set latitude = '42.6700491', longitude = '-83.0755758' where city = 'Shelby Township' and latitude is null;
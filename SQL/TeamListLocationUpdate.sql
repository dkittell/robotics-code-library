UPDATE
	fim_ftc_teams
SET
	latitude = (
		SELECT
			latitude
		FROM
			us_zip
		WHERE
			us_zip.city = fim_ftc_teams.city
		LIMIT
			1
	), longitude = (
		SELECT
			longitude
		FROM
			us_zip
		WHERE
			us_zip.city = fim_ftc_teams.city
		LIMIT
			1
	)
WHERE
	fim_ftc_teams.latitude IS NULL
	OR fim_ftc_teams.longitude IS NULL;

UPDATE
	fim_ftc_teams
SET
	latitude = (
		SELECT
			lat
		FROM
			us_cities
		WHERE
			us_cities.city = fim_ftc_teams.city
		LIMIT
			1
	), longitude = (
		SELECT
			lng
		FROM
			us_cities
		WHERE
			us_cities.city = fim_ftc_teams.city
		LIMIT
			1
	)
WHERE
	fim_ftc_teams.latitude IS NULL
	OR fim_ftc_teams.longitude IS NULL;

UPDATE
	fim_ftc_teams
SET
	latitude = 42.8471381,
	longitude = -83.0825578
WHERE
	city = 'Bruce Twp'
	AND (
		longitude IS NULL
		OR latitude IS NULL
	);

UPDATE
	fim_ftc_teams
SET
	latitude = 42.6700491,
	longitude = -83.0755758
WHERE
	city = 'Shelby Township'
	AND (
		longitude IS NULL
		OR latitude IS NULL
	);

INSERT INTO us_cities (city, state_id, lat, lng)
SELECT
	us_zip.city,
	us_zip.state_prefix,
	us_zip.latitude,
	us_zip.longitude
FROM
	us_zip
WHERE
	us_zip.state_prefix = 'mi'
	AND us_zip.city NOT IN (
		SELECT
			DISTINCT us_cities.city
		FROM
			us_cities
	)
SELECT
	*
FROM
	fim_ftc_teams
WHERE
	LENGTH(latitude) < 3
	OR latitude IS NULL
ORDER BY
	city;
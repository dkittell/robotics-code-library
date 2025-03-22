UPDATE
	fim_ftc_teams
SET
	fim_ftc_teams.latitude = (
		SELECT
			DISTINCT lat
		FROM
			us_cities
		WHERE
			us_cities.city = fim_ftc_teams.city
			AND us_cities.state_id = 'mi'
			limit 1

	)
WHERE
	fim_ftc_teams.city IS NOT NULL
	AND fim_ftc_teams.latitude IS NULL;

UPDATE
	fim_ftc_teams
SET
	fim_ftc_teams.longitude = (
		SELECT
			lng
		FROM
			us_cities
		WHERE
			us_cities.city = fim_ftc_teams.city
			AND us_cities.state_id = 'mi'
			limit 1
	)
WHERE
	fim_ftc_teams.city IS NOT NULL
	AND fim_ftc_teams.longitude IS NULL;

SELECT
	ftc.team_number,
	ftc.team_name,
	ftc.program,
	ftc.city,
	ftc.latitude,
	ftc.longitude,
	c.lat,
	c.lng
FROM
	fim_ftc_teams ftc
	LEFT OUTER JOIN us_cities c ON c.city = ftc.city
	AND c.state_id = 'mi'
WHERE
	ftc.city IS NOT NULL;
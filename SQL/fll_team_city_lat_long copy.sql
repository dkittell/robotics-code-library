UPDATE
	fim_fll_teams
SET
	fim_fll_teams.latitude = (
		SELECT
			DISTINCT lat
		FROM
			us_cities
		WHERE
			us_cities.city = fim_fll_teams.city
			AND us_cities.state_id = 'mi'
			limit 1

	)
WHERE
	fim_fll_teams.city IS NOT NULL
	AND fim_fll_teams.latitude IS NULL;

UPDATE
	fim_fll_teams
SET
	fim_fll_teams.longitude = (
		SELECT
			lng
		FROM
			us_cities
		WHERE
			us_cities.city = fim_fll_teams.city
			AND us_cities.state_id = 'mi'
			limit 1
	)
WHERE
	fim_fll_teams.city IS NOT NULL
	AND fim_fll_teams.longitude IS NULL;

SELECT
	fll.team_number,
	fll.team_name,
	fll.program,
	fll.city,
	fll.latitude,
	fll.longitude,
	c.lat,
	c.lng
FROM
	fim_fll_teams fll
	LEFT OUTER JOIN us_cities c ON c.city = fll.city
	AND c.state_id = 'mi'
WHERE
	fll.city IS NOT NULL;
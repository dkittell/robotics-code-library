
drop function if exists get_city_lat;
drop function if exists get_city_lon;

DELIMITER //
create FUNCTION get_city_lat(
	city_lookup VARCHAR(100)
) RETURNS DOUBLE 
DETERMINISTIC 
BEGIN 

RETURN(

		SELECT
			DISTINCT lat
		FROM
			us_cities
		WHERE
			city = city_lookup
			and state_id = 'mi'
		LIMIT 1
	); 
	
 END //
 DELIMITER ;
 
 
DELIMITER //
create FUNCTION get_city_lon(
	city_lookup VARCHAR(100)
) RETURNS DOUBLE 
DETERMINISTIC 
BEGIN 

RETURN(

		SELECT
			DISTINCT lng
		FROM
			us_cities
		WHERE
			city = city_lookup
			and state_id = 'mi'
		LIMIT 1
	); 
	
 END //
 DELIMITER ;
truncate table locations;

insert into locations (latitude, longitude,name, info)
SELECT latitude,longitude, concat(team_number, ' - ', team_name), concat(city, ' - ', affiliation) FROM fim_ftc_teams 
def utc_to_weekseconds(utc,leapseconds):
    """ Returns the GPS week, the GPS day, and the seconds
        and microseconds since the beginning of the GPS week
        https://stackoverflow.com/questions/45422739/gps-time-in-weeks-since-epoch-in-python
    """
    import datetime, calendar
    datetimeformat = "%Y-%m-%d %H:%M:%S"
    epoch = datetime.datetime.strptime("1980-01-06 00:00:00",datetimeformat)
    tdiff = utc -epoch  + datetime.timedelta(seconds=leapseconds)
    gpsweek = tdiff.days // 7
    gpsdays = tdiff.days - 7*gpsweek
    gpsseconds = tdiff.seconds + 86400* (tdiff.days -7*gpsweek)
    return gpsweek, gpsdays, gpsseconds, tdiff.microseconds


class GprmcMessage():
    """
    :see https://de.wikipedia.org/wiki/NMEA_0183
    """
    def __init__(self, datetime=None, status=None, lat=None, lat_ori=None, long=None, long_ori=None, vel=None,
                 course=None, mag=None, mag_sign=None, singularity=None, weeks=None, seconds=None):
        self.datetime = datetime
        self.status = status
        self.lat = lat
        self.lat_ori = lat_ori
        self.long = long
        self.long_ori = long_ori
        self.velocity = vel
        self.course = course
        self.mag = mag
        self.mag_sign = mag_sign
        self.singularity = singularity
        self.weeks = None
        self.seconds = None



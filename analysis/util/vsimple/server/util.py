# -*- coding: utf-8 -*-
import flask
__author__ = 'kaizhou'


def error_msg(msg):
    """
    This is a util function to return a Flask response with given message.
    :param msg: string
    :return: Flask.Response
    """
    return flask.jsonify(
        status="error",
        msg=msg
    )


def success_msg(data):
    """
    This is a util function to return a Flask response with given data.
    :param data: list or dict
    :return: Flask.Response
    """
    return flask.jsonify(
        status="success",
        data=data
    )
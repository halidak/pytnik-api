from django.urls import path
from .views import agent_path

urlpatterns = [
    path('get-path/', agent_path, name='agent_path'),
]

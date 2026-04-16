install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__init__.py" EXCLUDE
)

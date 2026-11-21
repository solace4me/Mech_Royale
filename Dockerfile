FROM python:3.13-slim
WORKDIR /bot
COPY bot.py mechfight.py precompute.py ./
COPY arena ./arena
# The map never changes: rasterize it once here (walls, roofs, ground) so the
# bot loads a ready grid instead of spending match time on it.
RUN python3 precompute.py arena /bot && chmod -R a+rX /bot
ENTRYPOINT ["python3", "-u", "/bot/bot.py"]

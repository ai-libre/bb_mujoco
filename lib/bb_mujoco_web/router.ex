defmodule BbMujocoWeb.Router do
  use BbMujocoWeb, :router

  pipeline :browser do
    plug :accepts, ["html"]
    plug :fetch_session
    plug :fetch_live_flash
    plug :put_root_layout, html: {BbMujocoWeb.Layouts, :root}
    plug :protect_from_forgery
    plug :put_secure_browser_headers
  end

  pipeline :api do
    plug :accepts, ["json", "xml"]
  end

  scope "/", BbMujocoWeb do
    pipe_through :browser

    get "/", PageController, :home

    # MuJoCo viewer
    live "/viewer/:robot", ViewerLive, :show
  end

  # MJCF API for serving robot models to browser
  scope "/api/mujoco", BbMujocoWeb do
    pipe_through :api

    get "/mjcf/:robot", MjcfController, :show
    get "/mjcf/:robot/assets", MjcfController, :list_assets
    get "/mjcf/:robot/assets/*path", MjcfController, :show_asset
  end
end
